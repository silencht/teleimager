import asyncio
import threading
import json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.contrib.media import MediaRelay
import av
import time
from fractions import Fraction
import numpy as np
from typing import Dict, Optional, Tuple, Any
import logging_mp
logger_mp = logging_mp.get_logger(__name__)


# ========================================================
# Embed HTML and JS directly
# ========================================================
INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>WebRTC webcam</title>
    <style>
    button {
        padding: 8px 16px;
    }

    video {
        width: 100%;
    }

    .option {
        margin-bottom: 8px;
    }

    #media {
        max-width: 1280px;
    }
    </style>
</head>
<body>

<div class="option">
    <input id="use-stun" type="checkbox"/>
    <label for="use-stun">Use STUN server</label>
</div>
<button id="start" onclick="start()">Start</button>
<button id="stop" style="display: none" onclick="stop()">Stop</button>

<div id="media">
    <h2>Media</h2>

    <audio id="audio" autoplay="true"></audio>
    <video id="video" autoplay="true" playsinline="true"></video>
</div>

<script src="client.js"></script>
</body>
</html>
"""

CLIENT_JS = """
var pc = null;

function negotiate() {
    pc.addTransceiver('video', { direction: 'recvonly' });
    pc.addTransceiver('audio', { direction: 'recvonly' });
    return pc.createOffer().then((offer) => {
        return pc.setLocalDescription(offer);
    }).then(() => {
        return new Promise((resolve) => {
            if (pc.iceGatheringState === 'complete') {
                resolve();
            } else {
                const checkState = () => {
                    if (pc.iceGatheringState === 'complete') {
                        pc.removeEventListener('icegatheringstatechange', checkState);
                        resolve();
                    }
                };
                pc.addEventListener('icegatheringstatechange', checkState);
            }
        });
    }).then(() => {
        var offer = pc.localDescription;
        return fetch('/offer', {
            body: JSON.stringify({
                sdp: offer.sdp,
                type: offer.type,
            }),
            headers: {
                'Content-Type': 'application/json'
            },
            method: 'POST'
        });
    }).then((response) => {
        return response.json();
    }).then((answer) => {
        return pc.setRemoteDescription(answer);
    }).catch((e) => {
        alert(e);
    });
}

function start() {
    var config = {
        sdpSemantics: 'unified-plan'
    };

    if (document.getElementById('use-stun').checked) {
        config.iceServers = [{ urls: ['stun:stun.l.google.com:19302'] }];
    }

    pc = new RTCPeerConnection(config);

    pc.addEventListener('track', (evt) => {
        if (evt.track.kind == 'video') {
            document.getElementById('video').srcObject = evt.streams[0];
        } else {
            document.getElementById('audio').srcObject = evt.streams[0];
        }
    });

    document.getElementById('start').style.display = 'none';
    negotiate();
    document.getElementById('stop').style.display = 'inline-block';
}

function stop() {
    document.getElementById('stop').style.display = 'none';
    setTimeout(() => { pc.close(); }, 500);
}
"""

# ========================================================
# Utility tools
# ========================================================
class TripleRingBuffer:
    def __init__(self):
        self.buffer = [None, None, None]
        self.write_index = 0            # Index where the next write will occur
        self.latest_index = -1          # Index of the latest written data
        self.read_index = -1            # Index of the current read data
        self.lock = threading.Lock()

    def write(self, data):
        with self.lock:
            self.buffer[self.write_index] = data
            self.latest_index = self.write_index
            self.write_index = (self.write_index + 1) % 3
            if self.write_index == self.read_index:
                self.write_index = (self.write_index + 1) % 3

    def read(self):
        with self.lock:
            if self.latest_index == -1:
                return None # No data has been written yet
            self.read_index = self.latest_index
        return self.buffer[self.read_index]

# ========================================================
# publish
# ========================================================
class BGRArrayVideoStreamTrack(MediaStreamTrack):
    """MediaStreamTrack exposing BGR ndarrays as av.VideoFrame (latest-frame semantics)."""
    kind = "video"

    def __init__(self):
        super().__init__()
        self._queue: asyncio.Queue = asyncio.Queue(maxsize=1)
        self._closed = False

    async def recv(self) -> av.VideoFrame:
        if self._closed:
            raise asyncio.CancelledError("Track has been closed")
        
        frame = await self._queue.get()
        if getattr(frame, "pts", None) is None:
            frame.pts = int(time.time() * 1000)
            frame.time_base = Fraction(1, 1000)
        return frame

    def _queue_put_nowait(self, frame: av.VideoFrame):
        try:
            self._queue.put_nowait(frame)
        except asyncio.QueueFull:
            # queue is full -> remove oldest and put newest
            try:
                _ = self._queue.get_nowait()
            except Exception:
                pass
            try:
                self._queue.put_nowait(frame)
            except Exception:
                # if still fails, silently drop
                pass

    def push_frame(self, bgr_numpy: np.ndarray, loop: Optional[asyncio.AbstractEventLoop] = None):
        """Push a BGR numpy array directly into the videotrack."""
        if bgr_numpy is None:
            return

        try:
            video_frame = av.VideoFrame.from_ndarray(bgr_numpy, format="bgr24")
        except Exception as e:
            logger_mp.debug("NDArrayVideoStreamTrack.push_frame: failed to convert ndarray to VideoFrame %s", e)
            return

        target_loop = loop or asyncio.get_event_loop()
        try:
            target_loop.call_soon_threadsafe(self._queue_put_nowait, video_frame)
        except Exception as e:
            logger_mp.debug("BGRArrayVideoStreamTrack.push_frame: scheduling error %s", e)

    async def stop(self):
        self._closed = True
        await super().stop()

class WebRTC_PublisherThread(threading.Thread):
    """A thread that runs an aiohttp web server with aiortc to publish video frames via WebRTC."""
    def __init__(self, port: int, host: str = "0.0.0.0"):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._loop: Optional[asyncio.AbstractEventLoop] = None
        self._app = web.Application()
        self._runner: Optional[web.AppRunner] = None
        self._pcs = set()  # set of active RTCPeerConnections
        self._start_event = threading.Event()
        self._stop_event = threading.Event()

        self._relay: Optional[MediaRelay] = None
        self._bgr_track: Optional[BGRArrayVideoStreamTrack] = None

        # register routes
        self._app.router.add_get("/", self._index)
        self._app.router.add_get("/client.js", self._javascript)
        self._app.router.add_post("/offer", self._offer)
        self._app.on_shutdown.append(self._on_shutdown)

    async def _index(self, request: web.Request) -> web.Response:
        return web.Response(content_type="text/html", text=INDEX_HTML)
    
    async def _javascript(self, request: web.Request) -> web.Response:
        return web.Response(content_type="application/javascript", text=CLIENT_JS)
    
    async def _offer(self, request: web.Request) -> web.Response:
        """Handle incoming WebRTC offer, create RTCPeerConnection, attach a _QueuedVideoTrack, and return an answer."""
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        # subscribe to relay-wrapped source track if available
        if self._relay and self._bgr_track:
            try:
                subscribed = self._relay.subscribe(self._bgr_track)
                pc.addTrack(subscribed)
            except Exception as e:
                logger_mp.warning("Failed to subscribe relay track: %s", e)
        else:
            logger_mp.warning("Relay or JPEG track not ready; peer will have no video")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            logger_mp.info(f"Connection state is {pc.connectionState}")
            if pc.connectionState == "failed" or pc.connectionState == "closed":
                await self._cleanup_pc(pc)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}),
        )

    async def _cleanup_pc(self, pc: RTCPeerConnection):
        """Cleanup a peer connection and its queue."""
        if pc in self._pcs:
            try:
                await pc.close()
            except Exception:
                pass
            self._pcs.discard(pc)

    async def _on_shutdown(self, app: web.Application):
        """Shutdown all pcs and its queues."""
        coros = [pc.close() for pc in list(self._pcs)]
        if coros:
            await asyncio.gather(*coros, return_exceptions=True)
        self._pcs.clear()
        if self._bgr_track:
            try:
                await self._bgr_track.stop()
            except Exception:
                pass
            self._bgr_track = None

    def wait_for_start(self, timeout: float = 1.0) -> bool:
        """Wait until http server is ready"""
        return self._start_event.wait(timeout=timeout)

    def run(self):
        async def _http_server():
            self._runner = web.AppRunner(self._app)
            await self._runner.setup()
            site = web.TCPSite(self._runner, self._host, self._port)
            await site.start()

            self._relay = MediaRelay()
            self._bgr_track = BGRArrayVideoStreamTrack()
            # mark started for external waiters
            self._start_event.set()

            while not self._stop_event.is_set():
                await asyncio.sleep(0.5)

        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        try:
            self._loop.run_until_complete(_http_server())
        finally:
            if self._bgr_track:
                try:
                    self._loop.run_until_complete(self._bgr_track.stop())
                except Exception:
                    pass
            if self._runner:
                try:
                    self._loop.run_until_complete(self._runner.cleanup())
                except Exception:
                    pass

            pending = asyncio.all_tasks(loop=self._loop)
            for t in pending:
                t.cancel()
            try:
                self._loop.run_until_complete(asyncio.gather(*pending, return_exceptions=True))
            finally:
                self._loop.close()

    def send(self, data):
        """Send data to all connected peers' queues."""
        if self._loop is None or self._bgr_track is None:
            return

        try:
            # pass the target loop to the push function so the track can schedule safely
            self._bgr_track.push_frame(data, loop=self._loop)
        except Exception as e:
            logger_mp.debug("Failed to push jpeg: %s", e)

    def stop(self):
        """Stop server and all peer connections"""
        self._stop_event.set()
        if self._loop:
            # trigger a dummy call to unblock loop sleep
            fut = asyncio.run_coroutine_threadsafe(asyncio.sleep(0), self._loop)
            try:
                fut.result(timeout=1)
            except Exception:
                pass
        self.join()

class PublisherManager:
    """Centralized management of WebRTC publishers"""

    _instance: Optional["PublisherManager"] = None
    _publisher_threads: Dict[Tuple[str, int], WebRTC_PublisherThread] = {}
    _lock = threading.Lock()
    _running = True

    def __init__(self):
        pass

    def _create_publisher_thread(self, port: int, host: str = "0.0.0.0") -> WebRTC_PublisherThread:
        try:
            publisher_thread = WebRTC_PublisherThread(port, host)
            publisher_thread.start()
            # Wait for the thread to start and socket to be ready
            if not publisher_thread.wait_for_start(timeout=1.0):
                raise ConnectionError(f"Publisher thread failed to start for {host}:{port}")

            return publisher_thread
        except Exception as e:
            logger_mp.error(f"Failed to create publisher thread for {host}:{port}: {e}")
            raise

    def _get_publisher_thread(self, port: int, host: str = "0.0.0.0") -> WebRTC_PublisherThread:
        key = (host, port)
        with self._lock:
            if key not in self._publisher_threads:
                self._publisher_threads[key] = self._create_publisher_thread(port, host)
            return self._publisher_threads[key]

    def _close_publisher(self, key: Tuple[str, int]) -> None:
        with self._lock:
            if key in self._publisher_threads:
                try:
                    self._publisher_threads[key].stop()
                except Exception as e:
                    logger_mp.error(f"Error stopping publisher at {key[0]}:{key[1]}: {e}")
                del self._publisher_threads[key]
    
    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    @classmethod
    def get_instance(cls) -> "PublisherManager":
        """Get or create the singleton instance with thread safety.
        Returns:
            The singleton WebRTCPublisherManager instance
        """
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def publish(self, data: Any, port: int, host: str = "0.0.0.0") -> None:
        """Publish data to queue-based communication.

        Args:
            data: The data to publish
            port: The port number
            host: The host address

        Raises:
            ConnectionError: If publishing fails
            SerializationError: If data serialization fails
        """
        if not self._running:
            raise RuntimeError("WebRTCPublisherManager is closed")

        try:
            publisher_thread = self._get_publisher_thread(port, host)
            publisher_thread.send(data)
        except Exception as e:
            logger_mp.error(f"Unexpected error in publish: {e}")
            raise

    def close(self) -> None:
        """Close all publishers."""
        self._running = False
        # close all publishers
        with self._lock:
            for key, publisher_thread in list(self._publisher_threads.items()):
                try:
                    publisher_thread.stop()
                except Exception as e:
                    logger_mp.error(f"Error stopping publisher at {key[0]}:{key[1]}: {e}")
            self._publisher_threads.clear()