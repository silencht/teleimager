import asyncio
import threading
import json
from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription, MediaStreamTrack
from aiortc.rtcrtpsender import RTCRtpSender
from aiortc.contrib.media import MediaRelay
from aiortc.codecs import h264
import av
import time
import ssl
import os
from pathlib import Path
import queue
import fractions
import numpy as np
from typing import Dict, Optional, Tuple, Any

import logging_mp
logger_mp = logging_mp.get_logger(__name__)

# ========================================================
# Path Configuration
# ========================================================
module_dir = Path(__file__).resolve().parent.parent.parent
default_cert = module_dir / "cert.pem"
default_key = module_dir / "key.pem"
env_cert = os.getenv("XR_TELEOP_CERT")
env_key = os.getenv("XR_TELEOP_KEY")
user_config_dir = Path.home() / ".config" / "xr_teleoperate"
user_cert = user_config_dir / "cert.pem"
user_key = user_config_dir / "key.pem"
CERT_PEM_PATH = Path(env_cert or (user_cert if user_cert.exists() else default_cert))
KEY_PEM_PATH = Path(env_key or (user_key if user_key.exists() else default_key))
CERT_PEM_PATH = CERT_PEM_PATH.resolve()
KEY_PEM_PATH = KEY_PEM_PATH.resolve()

# ========================================================
#  H264 NVENC Monkey Patch
# ========================================================
def nvenc_encode_frame(self, frame: av.VideoFrame, force_keyframe: bool):
    if self.codec and (frame.width != self.codec.width or frame.height != self.codec.height):
        self.codec = None

    if force_keyframe:
        frame.pict_type = av.video.frame.PictureType.I
    else:
        frame.pict_type = av.video.frame.PictureType.NONE

    if self.codec is None:
        # Try NVENC first, fallback to libx264 if needed (logic simplified here)
        try:
            self.codec = av.CodecContext.create("h264_nvenc", "w")
            logger_mp.debug(f"[H264 Patch] Initialized h264_nvenc encoder")
        except Exception:
            self.codec = av.CodecContext.create("libx264", "w")
            logger_mp.warning(f"[H264 Patch] NVENC failed, falling back to libx264")

        self.codec.width = frame.width
        self.codec.height = frame.height
        self.codec.bit_rate = self.target_bitrate
        self.codec.pix_fmt = "yuv420p"
        self.codec.framerate = fractions.Fraction(30, 1)
        self.codec.time_base = fractions.Fraction(1, 30)
        
        self.codec.options = {
            "preset": "fast",
            "zerolatency": "1",
            "g": "60",
            "delay": "0",
            "forced-idr": "1",
        }
        # If fallback to libx264, ensure threads=1 to save CPU
        if self.codec.name == "libx264":
            self.codec.options["level"] = "31"
            self.codec.options["preset"] = "ultrafast"
            self.codec.options["tune"] = "zerolatency"
            self.codec.options["threads"] = "1"

        self.frame_count = 0
        force_keyframe = True

    if not force_keyframe:
        if hasattr(self, "frame_count") and self.frame_count % 60 == 0:
            force_keyframe = True
    
    if hasattr(self, "frame_count"):
        self.frame_count += 1
    else:
        self.frame_count = 1

    if force_keyframe:
        frame.pict_type = av.video.frame.PictureType.I
    else:
        frame.pict_type = av.video.frame.PictureType.NONE

    try:
        for packet in self.codec.encode(frame):
            data = bytes(packet)
            if data:
                yield from self._split_bitstream(data)
    except Exception as e:
        logger_mp.warning(f"[H264 Patch] Encode error: {e}")

logger_mp.info("[System] Applying H264 NVENC Monkey Patch to aiortc...")
h264.H264Encoder._encode_frame = nvenc_encode_frame

# ========================================================
# Embed HTML and JS directly
# ========================================================
# ========================================================
# Embed HTML and JS directly
# ========================================================
INDEX_HTML = """
<!DOCTYPE html>
<html>
<head>
    <meta charset="UTF-8"/>
    <meta name="viewport" content="width=device-width, initial-scale=1.0" />
    <title>WebRTC Stream</title>
    <style>
    body { 
        font-family: sans-serif; 
        background: #fff; 
        color: #000; 
        text-align: center; 
    }
    button { padding: 10px 20px; font-size: 16px; cursor: pointer; }
    video { width: 100%; max-width: 1280px; background: #000; margin-top: 10px; }
    
    /* Title link style */
    h1 a {
        text-decoration: none;
        color: #000;
    }
    h1 a:hover {
        color: #555;
    }
    </style>
</head>
<body>
    <h1>
        <a href="https://github.com/unitreerobotics/teleimager" target="_blank">
            XR Teleoperation WebRTC Camera Stream
        </a>
    </h1>

    <div style="margin-bottom: 20px;">
        <a href="https://www.unitree.com/" target="_blank">
            <img src="https://www.unitree.com/images/0079f8938336436e955ea3a98c4e1e59.svg" alt="Unitree LOGO" width="10%">
        </a>
    </div>

    <button id="start" onclick="start()">Start</button>
    <button id="stop" style="display: none" onclick="stop()">Stop</button>
    
    <div id="media">
        <video id="video" autoplay playsinline muted></video>
        <audio id="audio" autoplay></audio>
    </div>
    
    <script src="client.js"></script>
</body>
</html>
"""

CLIENT_JS = """
var pc = null;

function negotiate() {
    pc.addTransceiver('video', { direction: 'recvonly' });
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

    // Removed STUN server check logic completely

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
    document.getElementById('start').style.display = 'inline-block';
    if (pc) {
        pc.close();
        pc = null;
    }
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
        self._start_time = None
        self._pts = 0

    async def recv(self) -> av.VideoFrame:
        # This will suspend execution until a frame is available
        # preventing CPU busy-waiting
        frame = await self._queue.get()
        return frame

    def push_frame(self, bgr_numpy: np.ndarray, loop: Optional[asyncio.AbstractEventLoop] = None):
        if bgr_numpy is None:
            return

        # 1. Convert and calculate PTS immediately
        # MediaRelay requires consistent PTS to function correctly
        try:
            video_frame = av.VideoFrame.from_ndarray(bgr_numpy, format="bgr24")
            
            if self._start_time is None:
                self._start_time = time.time()
                self._pts = 0
            else:
                # 90000 is the standard RTP clock rate for video
                # This ensures smooth playback
                self._pts = int((time.time() - self._start_time) * 90000)
            
            video_frame.pts = self._pts
            video_frame.time_base = fractions.Fraction(1, 90000)
            
        except Exception as e:
            logger_mp.debug(f"Conversion failed: {e}")
            return

        # 2. Push to queue thread-safely
        target_loop = loop or asyncio.get_event_loop()
        if target_loop.is_closed():
            return
            
        def _put():
            try:
                # Drop old frame if queue is full (Low Latency strategy)
                if self._queue.full():
                    self._queue.get_nowait()
                self._queue.put_nowait(video_frame)
            except Exception:
                pass

        target_loop.call_soon_threadsafe(_put)


class WebRTC_PublisherThread(threading.Thread):
    """
    Runs aiohttp + aiortc in a separate THREAD (not Process).
    This enables shared memory and removes Pickling overhead.
    """
    def __init__(self, port: int, host: str = "0.0.0.0", codec_pref: str = None):
        super().__init__(daemon=True)
        self._host = host
        self._port = port
        self._codec_pref = codec_pref
        self._app = web.Application()
        self._runner: Optional[web.AppRunner] = None
        self._pcs = set()
        self._start_event = threading.Event()
        self._stop_event = threading.Event()
        self._frame_queue = queue.Queue(maxsize=1)

        self._bgr_track: Optional[BGRArrayVideoStreamTrack] = None
        self._relay: Optional[MediaRelay] = None
        self._loop: Optional[asyncio.AbstractEventLoop] = None

        # register routes
        self._app.router.add_get("/", self._index)
        self._app.router.add_get("/client.js", self._javascript)
        self._app.router.add_post("/offer", self._offer)

        self._app.router.add_options("/", self._options)
        self._app.router.add_options("/client.js", self._options)
        self._app.router.add_options("/offer", self._options)

    async def _index(self, request: web.Request) -> web.Response:
        return web.Response(content_type="text/html", text=INDEX_HTML)
    
    async def _javascript(self, request: web.Request) -> web.Response:
        return web.Response(content_type="application/javascript", text=CLIENT_JS)

    async def _options(self, request):
        return web.Response(headers={
            "Access-Control-Allow-Origin": "*",
            "Access-Control-Allow-Methods": "POST, GET, OPTIONS",
            "Access-Control-Allow-Headers": "Content-Type",
        })

    async def _offer(self, request: web.Request) -> web.Response:
        params = await request.json()
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self._pcs.add(pc)

        # CORE LOGIC: Use MediaRelay to subscribe
        # This ensures encoding happens only once globally
        if self._bgr_track and self._relay:
            try:
                relayed_track = self._relay.subscribe(self._bgr_track)
                transceiver = pc.addTransceiver(relayed_track, direction="sendonly")
                
                if self._codec_pref:
                    capabilities = RTCRtpSender.getCapabilities("video")
                    pref = self._codec_pref.lower()
                    
                    if pref == "h264":
                        codecs = [c for c in capabilities.codecs if c.mimeType == "video/H264"]
                        if codecs: 
                            transceiver.setCodecPreferences(codecs)
                            logger_mp.info(f"Applied H264 codec preference for WebRTC Publisher on:{self._port}")
                            
                    elif pref == "vp8":
                        codecs = [c for c in capabilities.codecs if c.mimeType == "video/VP8"]
                        if codecs: 
                            transceiver.setCodecPreferences(codecs)
                            logger_mp.info(f"Applied VP8 codec preference for WebRTC Publisher on:{self._port}")
                            
                    else:
                        logger_mp.warning(f"Unknown codec preference: {pref}. Using auto-negotiation on:{self._port}")
                else:
                    logger_mp.info(f"No codec preference set. Using auto-negotiation on:{self._port}")
                    
            except Exception as e:
                logger_mp.error(f"Relay subscription failed: {e}")

        @pc.on("connectionstatechange")
        async def on_connectionstatechange():
            if pc.connectionState in ["failed", "closed"]:
                await self._cleanup_pc(pc)

        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.Response(
            content_type="application/json",
            text=json.dumps({"sdp": pc.localDescription.sdp, "type": pc.localDescription.type}),
            headers={"Access-Control-Allow-Origin": "*"}
        )

    async def _cleanup_pc(self, pc):
        self._pcs.discard(pc)
        try:
            await pc.close()
        except: pass

    def wait_for_start(self, timeout=1.0):
        return self._start_event.wait(timeout=timeout)

    def run(self):
        # Create a new Event Loop for this thread
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)
        
        async def _main():
            self._runner = web.AppRunner(self._app)
            await self._runner.setup()
            
            # Init Track and Relay inside the loop
            self._bgr_track = BGRArrayVideoStreamTrack()
            self._relay = MediaRelay()

            ssl_context = ssl.SSLContext(ssl.PROTOCOL_TLS_SERVER)
            ssl_context.load_cert_chain(CERT_PEM_PATH, KEY_PEM_PATH)
            site = web.TCPSite(self._runner, self._host, self._port, ssl_context=ssl_context)
            await site.start()
            self._start_event.set()
            
            # Frame Pushing Loop
            while not self._stop_event.is_set():
                try:
                    # Non-blocking check for new frames
                    if not self._frame_queue.empty():
                        # Get frame (no pickling overhead in Threads!)
                        frame = self._frame_queue.get_nowait()
                        self._bgr_track.push_frame(frame, loop=self._loop)
                    
                    # CRITICAL: Yield control to asyncio loop to handle WebRTC packets
                    await asyncio.sleep(0.005)
                except Exception:
                    await asyncio.sleep(0.005)

        try:
            self._loop.run_until_complete(_main())
        except Exception as e:
            logger_mp.error(f"WebRTC Thread Error: {e}")
        finally:
            if self._loop: self._loop.close()

    def send(self, data: np.ndarray):
        """Send data to the processing thread."""
        # Simple drop-frame logic if queue is full
        if not self._frame_queue.full():
            self._frame_queue.put(data)
        else:
            try:
                self._frame_queue.get_nowait()
                self._frame_queue.put(data)
            except: pass

    def stop(self):
        self._stop_event.set()
        self.join(timeout=1.0)


# ========================================================
# Manager (Singleton)
# ========================================================
class PublisherManager:
    """Manages WebRTC_PublisherThreads."""
    _instance: Optional["PublisherManager"] = None
    _publisher_threads: Dict[Tuple[str, int], WebRTC_PublisherThread] = {}
    _lock = threading.Lock()
    _running = True

    def __init__(self):
        pass

    @classmethod
    def get_instance(cls) -> "PublisherManager":
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = cls()
        return cls._instance

    def _create_publisher(self, port: int, host: str, codec_pref: str):
        t = WebRTC_PublisherThread(port, host, codec_pref)
        t.start()
        if not t.wait_for_start(timeout=5.0):
             raise ConnectionError("Publisher failed to start (Timeout)")
        return t

    def _get_publisher(self, port, host, codec_pref):
        key = (host, port)
        with self._lock:
            if key not in self._publisher_threads:
                self._publisher_threads[key] = self._create_publisher(port, host, codec_pref)
            return self._publisher_threads[key]

    def publish(self, data: Any, port: int, host: str = "0.0.0.0", codec_pref: str = None) -> None:
        if not self._running: return
        try:
            pub = self._get_publisher(port, host, codec_pref)
            pub.send(data)
        except Exception as e:
            logger_mp.error(f"Unexpected error in publish: {e}")
            pass

    def close(self) -> None:
        self._running = False
        with self._lock:
            for key, pub in list(self._publisher_threads.items()):
                try:
                    pub.stop()
                except Exception: pass
            self._publisher_threads.clear()