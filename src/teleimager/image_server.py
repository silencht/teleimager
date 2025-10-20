import os
import argparse
import glob
import cv2
import numpy as np
import uvc
import yaml
import time
import messaging
import threading
import signal
import functools
import subprocess
import logging_mp
logger_mp = logging_mp.get_logger(__name__, level=logging_mp.INFO)

# Resolve the absolute path of cam_config.yaml relative to this script
CONFIG_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "cam_config.yaml"                
)
CONFIG_PATH = os.path.normpath(CONFIG_PATH)

# ========================================================
# camera and camera discovery
# ========================================================
class CameraFinder:
    """ 
    Discover connected cameras and their properties.
    vpath: /dev/videoX
    ppath: physical path in /sys/class/video4linux, e.g. /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.2/1-11.2:1.0
    uid: USB unique ID, e.g. "001:002"
    dev_info: extra info from uvc
    sn: serial number of the camera
    """
    def __init__(self, realsense_enable=False, verbose=False):
        self.verbose = verbose
        # uvc
        self._reload_uvc_driver()
        self.uvc_devices = uvc.device_list()
        self.uid_map = {dev["uid"]: dev for dev in self.uvc_devices}
        # all video devices
        self.video_paths = self._list_video_paths()
        # realsense
        if realsense_enable:
            self.rs_serial_numbers = self._list_realsense_serial_numbers()
            self.rs_video_paths = self._list_realsense_video_paths()
            self.rs_rgb_video_paths = [p for p in self.rs_video_paths if self._is_like_rgb(p)]
        else:
            self.rs_serial_numbers = []
            self.rs_video_paths = []
            self.rs_rgb_video_paths = []
        # rgb & uvc
        self.uvc_rgb_video_paths = self._list_uvc_rgb_video_paths()
        self.uvc_rgb_video_ids = [int(v.replace("/dev/video", "")) for v in self.uvc_rgb_video_paths]
        self.uvc_rgb_physical_paths = [self._get_ppath_from_vpath(v) for v in self.uvc_rgb_video_paths]
        self.uvc_rgb_uids = [self._get_uid_from_ppath(p) for p in self.uvc_rgb_physical_paths]
        self.uvc_rgb_dev_info = [self.uid_map.get(uid) for uid in self.uvc_rgb_uids]
        self.uvc_rgb_serial_numbers = [dev_info.get("serialNumber") if dev_info else None for dev_info in self.uvc_rgb_dev_info]
        # all uvc cameras
        self.uvc_rgb_cameras = {}
        for vpath, vid, ppath, uid, dev_info, sn in zip(
            self.uvc_rgb_video_paths,
            self.uvc_rgb_video_ids,
            self.uvc_rgb_physical_paths,
            self.uvc_rgb_uids,
            self.uvc_rgb_dev_info,
            self.uvc_rgb_serial_numbers,
        ):
            self.uvc_rgb_cameras[vpath] = {
                "video_id": vid,
                "physical_path": ppath,
                "uid": uid,
                "dev_info": dev_info,
                "serial_number": sn
            }
        if self.verbose:
            self.info()

    # utils
    def _reload_uvc_driver(self):
        try:
            subprocess.run("sudo modprobe -r uvcvideo", shell=True, check=True)
            time.sleep(1)
            subprocess.run("sudo modprobe uvcvideo debug=0", shell=True, check=True)
            time.sleep(1)
        except subprocess.CalledProcessError as e:
            logger_mp.error(f"Failed to reload driver: {e}")

    def _list_video_paths(self):
        base = "/sys/class/video4linux/"
        if not os.path.exists(base):
            return []
        return [f"/dev/{x}" for x in sorted(os.listdir(base)) if x.startswith("video")]

    def _list_uvc_rgb_video_paths(self):
        return [p for p in self.video_paths if self._is_like_rgb(p) and p not in self.rs_video_paths]

    def _list_realsense_video_paths(self):
        def _read_text(path):
            try:
                with open(path, "r", encoding="utf-8", errors="ignore") as f:
                    return f.read().strip()
            except Exception:
                return None

        def _parent_usb_device_sysdir(video_sysdir):
            d = os.path.realpath(os.path.join(video_sysdir, "device"))
            for _ in range(10):
                if d is None or d == "/" or not os.path.isdir(d):
                    break
                id_vendor = _read_text(os.path.join(d, "idVendor"))
                id_product = _read_text(os.path.join(d, "idProduct"))
                if id_vendor and id_product:
                    return d
                d_next = os.path.dirname(d)
                if d_next == d:
                    break
                d = d_next
            return None

        ports = []
        for devnode in sorted(glob.glob("/dev/video*")):
            sysdir = f"/sys/class/video4linux/{os.path.basename(devnode)}"
            name = _read_text(os.path.join(sysdir, "name"))
            usb_dir = _parent_usb_device_sysdir(sysdir)
            vendor_id = _read_text(os.path.join(usb_dir, "idVendor")) if usb_dir else None

            # Match RealSense by name and Intel vendor ID
            if name and "realsense" in name.lower() and (vendor_id or "").lower() in ("8086", "32902"):
                ports.append(devnode)

        return ports

    def _list_realsense_serial_numbers(self):
        try:
            import pyrealsense2 as rs
        except ImportError:
            raise RuntimeError(
                "[RealSenseCamera] pyrealsense2 is not installed. Please install it to use RealSense cameras.\n"
                "You can install it manually with:\n"
                "    cd ~\n"
                "    git clone https://github.com/IntelRealSense/librealsense.git\n"
                "    cd librealsense\n"
                "    git checkout v2.50.0\n"
                "    mkdir build && cd build\n"
                "    cmake .. -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=$(which python3)\n"
                "    make -j$(nproc)\n"
                "    sudo make install\n"
            )
        ctx = rs.context()
        devices = ctx.query_devices()
        serials = []
        for dev in devices:
            try:
                serials.append(dev.get_info(rs.camera_info.serial_number))
            except Exception:
                continue
        return serials

    def _get_ppath_from_vpath(self, video_path):
        sysfs_path = f"/sys/class/video4linux/{os.path.basename(video_path)}/device"
        return os.path.realpath(sysfs_path)

    def _get_uid_from_ppath(self, physical_path):
        def read_file(path):
            return open(path).read().strip() if os.path.exists(path) else None

        busnum_file = os.path.join(physical_path, "busnum")
        devnum_file = os.path.join(physical_path, "devnum")

        if not (os.path.exists(busnum_file) and os.path.exists(devnum_file)):
            parent = os.path.dirname(physical_path)
            busnum_file = os.path.join(parent, "busnum")
            devnum_file = os.path.join(parent, "devnum")

        if os.path.exists(busnum_file) and os.path.exists(devnum_file):
            bus = read_file(busnum_file)
            dev = read_file(devnum_file)
            return f"{bus}:{dev}"
        return None

    def _is_like_rgb(self, video_path):
        cap = cv2.VideoCapture(video_path)
        if not cap.isOpened():
            return False
        ret, frame = cap.read()
        cap.release()
        return ret and frame is not None and frame.ndim == 3 and frame.shape[2] == 3

    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    def is_rs_serial_exist(self, serial_number):
        return str(serial_number) in self.rs_serial_numbers

    def is_vpath_exist(self, vpath):
        return vpath in self.video_paths
    
    def is_ppath_exist(self, physical_path):
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("physical_path") == physical_path:
                return True
        return False
    
    def get_uid_by_sn(self, serial_number):
        matches = [
            cam for cam in self.uvc_rgb_cameras.values()
            if cam.get("serial_number") == str(serial_number)
        ]
        if not matches:
            return None
        if len(matches) > 1:
            raise ValueError(f"Multiple cameras found with serial number {serial_number}")
        return matches[0].get("uid")

    def get_uid_by_ppath(self, physical_path):
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("physical_path") == physical_path:
                return cam.get("uid")
        return None
    
    def get_uid_by_vpath(self, video_path):
        cam = self.uvc_rgb_cameras.get(video_path)
        if cam:
            return cam.get("uid")
        return None
    
    def get_vpath_by_sn(self, serial_number):
        matches = []
        for cam in self.uvc_rgb_cameras.values():
            if cam.get("serial_number") == str(serial_number):
                vpath = f"/dev/video{cam.get('video_id')}"
                matches.append(vpath)
        if not matches:
            return None
        if len(matches) > 1:
            raise ValueError(f"Multiple video devices found for serial number {serial_number}: {matches}. ")
        return matches[0]

    def get_vpath_by_ppath(self, physical_path):
        base = "/sys/class/video4linux/"
        matches = []
        for v in os.listdir(base):
            sys_path = os.path.realpath(os.path.join(base, v, "device"))
            if sys_path == physical_path:
                vpath = f"/dev/{v}"
                if self._is_like_rgb(vpath):
                    matches.append(vpath)
        if not matches:
            return None
        if len(matches) > 1:
            raise ValueError(f"Multiple video devices found for physical path {physical_path}: {matches}. ")
        return matches[0]
    

    def info(self):
        logger_mp.info("======================= Camera Discovery Start ==================================")
        logger_mp.info("Found video devices: %s", self.video_paths)
        logger_mp.info("Found RGB video devices: %s", self.uvc_rgb_video_paths)

        if self.rs_serial_numbers:
            logger_mp.info("----------------------- Realsense Cameras ----------------------------------")
            logger_mp.info(f"RealSense serial numbers: {self.rs_serial_numbers}")
            logger_mp.info(f"RealSense video paths: {self.rs_video_paths}")
            logger_mp.info(f"RealSense RGB-like video paths: {self.rs_rgb_video_paths}")

        for idx, (vpath, cam) in enumerate(self.uvc_rgb_cameras.items(), start=1):
            logger_mp.info("----------------------- OpenCV / UVC Camera %d -----------------------------", idx)
            logger_mp.info("video_path    : %s", vpath)
            logger_mp.info("video_id      : %s", cam.get("video_id"))
            logger_mp.info("serial_number : %s", cam.get("serial_number") or "unknown")
            logger_mp.info("physical_path : %s", cam.get("physical_path"))
            logger_mp.info("extra_info:")

            dev_info = cam.get("dev_info")
            uid = cam.get("uid")

            if dev_info:
                for k, v in dev_info.items():
                    logger_mp.info("    %s: %s", k, v)
                try:
                    cap = uvc.Capture(uid)
                    for fmt in cap.available_modes:
                        logger_mp.info("    format: %dx%d@%d %s", fmt.height, fmt.width, fmt.fps, fmt.format_name)
                    cap.close()
                    cap = None
                except Exception as e:
                    logger_mp.warning("    failed to get formats: %s", e)
            else:
                logger_mp.info("    no uvc extra info available")

        logger_mp.info("=========================== Camera Discovery End ================================")

class BaseCamera:
    def __init__(self, img_shape, fps, port):
        self.img_shape = img_shape # (H, W)
        self.fps = fps
        self.port = port

    def __str__(self):
        raise NotImplementedError
    
    def __repr__(self):
        return self.__str__()

    def get_frame(self):
        """Return a color image frame as bytes"""
        raise NotImplementedError

    def get_depth_frame(self):
        """Return a depth frame as bytes, or None if not supported"""
        return None

    def get_port(self):
        """Return the port number the camera is serving on."""
        return self.port

    def get_fps(self):
        """Return the camera FPS setting."""
        return self.fps

    def release(self):
        """Release camera resources."""
        raise NotImplementedError

class RealSenseCamera(BaseCamera):
    def __init__(self, serial_number, img_shape, fps, port, enable_depth=False) -> None:
        super().__init__(img_shape, fps, port)
        self.serial_number = serial_number
        self.enable_depth = enable_depth
        self._latest_depth = None

        try:
            import pyrealsense2 as rs
        except ImportError:
            raise RuntimeError(
                "[RealSenseCamera] pyrealsense2 is not installed. Please install it to use RealSense cameras.\n"
                "You can install it manually with:\n"
                "    cd ~\n"
                "    git clone https://github.com/IntelRealSense/librealsense.git\n"
                "    cd librealsense\n"
                "    git checkout v2.50.0\n"
                "    mkdir build && cd build\n"
                "    cmake .. -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE=$(which python3)\n"
                "    make -j$(nproc)\n"
                "    sudo make install\n"
            )
        try:
            align_to = rs.stream.color
            self.align = rs.align(align_to)
            self.pipeline = rs.pipeline()
            config = rs.config()
            config.enable_device(self.serial_number)

            config.enable_stream(rs.stream.color, self.img_shape[1], self.img_shape[0], rs.format.bgr8, self.fps)
            if self.enable_depth:
                config.enable_stream(rs.stream.depth, self.img_shape[1], self.img_shape[0], rs.format.z16, self.fps)

            profile = self.pipeline.start(config)
            self._device = profile.get_device()
            if self._device is None:
                logger_mp.error('[RealSenseCamera] pipe_profile.get_device() is None .')
            if self.enable_depth:
                assert self._device is not None
                depth_sensor = self._device.first_depth_sensor()
                self.g_depth_scale = depth_sensor.get_depth_scale()

            self.intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()
            logger_mp.info(f"[RealSenseCamera] {self.serial_number} initialized with "
                        f"{self.img_shape[0]} x {self.img_shape[1]} @ {self.fps}, on port: {self.port}")
        except Exception as e:
            if self.pipeline:
                try:
                    self.pipeline.stop()
                except:
                    pass
            raise RuntimeError(f"[RealSenseCamera] Failed to initialize RealSense camera {self.serial_number}: {e}")

    def __str__(self):
        return f"RealSenseCamera(SN:{self.serial_number}, port={self.port}, shape={self.img_shape}, fps={self.fps})"

    def get_frame(self):
        frames = self.pipeline.wait_for_frames()
        aligned_frames = self.align.process(frames)
        color_frame = aligned_frames.get_color_frame()
        if not color_frame:
            return None

        if self.enable_depth:   
            depth_frame = aligned_frames.get_depth_frame()
            if depth_frame:
                self._latest_depth = np.asanyarray(depth_frame.get_data())
            else:
                self._latest_depth = None

        color_image = np.asanyarray(color_frame.get_data())
        # color_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB)
        ok, buf = cv2.imencode(".jpg", color_image)
        return buf.tobytes() if ok else None

    def get_depth_frame(self):
        if self._latest_depth is None:
            return None
        return self._latest_depth.tobytes()

    def release(self):
        self.pipeline.stop()
        logger_mp.info(f"[RealSenseCamera] Released {self.serial_number}")

class UVCCamera(BaseCamera):
    def __init__(self, uid, img_shape, fps, port):
        super().__init__(img_shape, fps, port)
        self.uid = uid
        self.cap = None
        try:
            self.cap = uvc.Capture(self.uid)
        except Exception as e:
            self.cap = None
            raise RuntimeError(f"[UVCCamera] Failed to open camera {self.uid}: {e}")

        try:
            self.cap.frame_mode = self._choose_mode(self.cap, width=self.img_shape[1], height=self.img_shape[0], fps=self.fps)
            logger_mp.info(f"[UVCCamera] {self.uid} initialized "
                           f"with {self.img_shape[0]} x {self.img_shape[1]} @ {self.fps}  MJPG, on port: {self.port}")
        except Exception as e:
            self.cap = None
            raise RuntimeError(f"[UVCCamera] Failed to set mode for {self.uid}: {e}")

    def __str__(self):
        return f"UVCCamera(UID:{self.uid}, port={self.port}, shape={self.img_shape}, fps={self.fps})"

    def _choose_mode(self, cap, width=None, height=None, fps=None):
        for m in cap.available_modes:
            if m.width == width and m.height == height and m.fps == fps and m.format_name == "MJPG":
                return m
        raise ValueError("[UVCCamera] No matching uvc mode found")

    def get_frame(self):
        if self.cap is None:
            return None
        frame = self.cap.get_frame() # get_frame_robust()
        if frame is None or frame.jpeg_buffer is None:
            return None
        return bytes(frame.jpeg_buffer)

    def release(self):
        if self.cap:
            try:
                self.cap.close()
            except Exception as e:
                logger_mp.error(f"Error releasing UVC camera: {e}")
            time.sleep(0.3)
            self.cap = None
        logger_mp.info(f"[UVCCamera] Released {self.uid}")

class OpenCVCamera(BaseCamera):
    def __init__(self, video_path, img_shape, fps, port):
        super().__init__(img_shape, fps, port)
        self.video_path = video_path

        self.cap = cv2.VideoCapture(self.video_path, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.img_shape[0])
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH,  self.img_shape[1])
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        # Test if the camera can read frames
        if not self._can_read_frame():
            self.release()
            raise RuntimeError(f"[OpenCVCamera] Camera {self.video_path} failed to initialize or read frames.")
        else:
            logger_mp.info(f"[OpenCVCamera] {self.video_path} initialized with "
                           f"{self.img_shape[0]} x {self.img_shape[1]} @ {self.fps}, on port: {self.port}")
    def __str__(self):
        return f"OpenCVCamera(path:{self.video_path}, port={self.port}, shape={self.img_shape}, fps={self.fps})"
        
    def _can_read_frame(self):
        success, _ = self.cap.read()
        return success

    def get_frame(self):
        ret, frame = self.cap.read()
        if not ret or frame is None:
            return None
        ok, buf = cv2.imencode(".jpg", frame)
        return buf.tobytes() if ok else None

    def release(self):
        self.cap.release()
        self.cap = None
        logger_mp.info(f"[OpenCVCamera] Released {self.video_path}")

# ========================================================
# image server
# ========================================================
class ImageServer:
    def __init__(self, cam_config, realsense_enable=False, camera_finder_verbose=False):
        self._cam_config = cam_config
        self._realsense_enable = realsense_enable
        self._stop_event = threading.Event()
        self._cameras = {}
        self._cam_finder = CameraFinder(realsense_enable, camera_finder_verbose)
        self._responser = messaging.Responser(self._cam_config)
        self._publisher_manager = messaging.PublisherManager.get_instance()
        self._publisher_threads = []  # keep references for graceful join

        try:
            # Load cameras from self.cam_config
            for cam_topic, cam_cfg in self._cam_config.items():
                if not cam_cfg.get("enable", False):
                    continue

                cam_type = cam_cfg["type"]
                cam_port = cam_cfg["port"]
                img_shape = cam_cfg["image_shape"]
                fps = cam_cfg["fps"]
                video_id = cam_cfg["video_id"]
                video_path = f"/dev/video{video_id}" if video_id else None
                physical_path = str(cam_cfg.get("physical_path")) if cam_cfg.get("physical_path") else None
                serial_number = str(cam_cfg.get("serial_number")) if cam_cfg.get("serial_number") else None

                if cam_type == "opencv":
                    if physical_path is not None:
                        vpath = self._cam_finder.get_vpath_by_ppath(physical_path)
                        if vpath is None:
                            self._cameras[cam_topic] = None
                            logger_mp.error(f"[Image Server] Cannot find OpenCVCamera for {cam_topic} with physical path {physical_path}")
                        else:
                            self._cameras[cam_topic] = OpenCVCamera(vpath, img_shape, fps, cam_port)
                            continue

                    if serial_number is not None:
                        vpath = self._cam_finder.get_vpath_by_sn(serial_number)
                        if vpath is None:
                            self._cameras[cam_topic] = None
                            logger_mp.error(f"[Image Server] Cannot find OpenCVCamera for {cam_topic} with serial number {serial_number}")
                        else:
                            self._cameras[cam_topic] = OpenCVCamera(vpath, img_shape, fps, cam_port)
                        # once you specify either `physical_path` or `serial_number`, the system will no longer fall back to searching by `video_id`.
                        # ——— even if no camera matches the given path/serial.
                        continue
                    
                    if not self._cam_finder.is_vpath_exist(video_path):
                        self._cameras[cam_topic] = None
                        logger_mp.error(f"[Image Server] Cannot find OpenCVCamera for {cam_topic} with video_id {video_id}")
                    else:
                        self._cameras[cam_topic] = OpenCVCamera(video_path, img_shape, fps, cam_port)
                        

                elif cam_type == "realsense":
                    if not self._realsense_enable:
                        self._cameras[cam_topic] = None
                        logger_mp.error(f"[Image Server] Please start image server with the '--rs' flag to support Realsense {cam_topic}.")
                    elif not self._cam_finder.is_rs_serial_exist(serial_number):
                        self._cameras[cam_topic] = None
                        logger_mp.error(f"[Image Server] Cannot find RealSenseCamera for {cam_topic}")
                    else:
                        self._cameras[cam_topic] = RealSenseCamera(serial_number, img_shape, fps, cam_port)

                elif cam_type == "uvc":
                    uid = None
                    if physical_path is not None:
                        uid = self._cam_finder.get_uid_by_ppath(physical_path)
                        if uid is None:
                            self._cameras[cam_topic] = None
                            logger_mp.error(f"[Image Server] Cannot find UVCCamera for {cam_topic} with physical path {physical_path}")
                        else:
                            self._cameras[cam_topic] = UVCCamera(uid, img_shape, fps, cam_port)
                            continue

                    if serial_number is not None:
                        uid = self._cam_finder.get_uid_by_sn(serial_number)
                        if uid is None:
                            self._cameras[cam_topic] = None
                            logger_mp.error(f"[Image Server] Cannot find UVCCamera for {cam_topic} with serial number {serial_number}")
                        else:
                            self._cameras[cam_topic] = UVCCamera(uid, img_shape, fps, cam_port)
                        # once you specify either `physical_path` or `serial_number`, the system will no longer fall back to searching by `video_id`.
                        # ——— even if no camera matches the given path/serial.
                        continue

                    if video_id is not None:
                        if not self._cam_finder.is_vpath_exist(video_path):
                            self._cameras[cam_topic] = None
                            logger_mp.error(f"[Image Server] Cannot find UVCCamera for {cam_topic} with video_id {video_id}")
                        else:
                            uid = self._cam_finder.get_uid_by_vpath(video_path)
                            if uid is None:
                                self._cameras[cam_topic] = None
                                logger_mp.error(f"[Image Server] Cannot find UVCCamera for {cam_topic} with uid from video_id {video_id}")
                            else:
                                self._cameras[cam_topic] = UVCCamera(uid, img_shape, fps, cam_port)
                else:
                    logger_mp.error(f"[Image Server] Unknown camera type {cam_type} for {cam_topic}, skipping...")
                    continue
        except Exception as e:
            logger_mp.error(f"[Image Server] Initialization failed: {e}")
            self._clean_up()
            raise

        logger_mp.info("[Image Server] Image server has started, waiting for client connections...")
 
    def _image_pub(self, camera_topic_name, camera, stop_event):
        try:
            interval = 1.0 / camera.get_fps()
            next_frame_time = time.monotonic()

            while not stop_event.is_set():
                frame = camera.get_frame()
                if frame:
                    self._publisher_manager.publish(frame, camera.get_port())
                else:
                    logger_mp.warning(f"Camera {camera} returned no frame.")
                    stop_event.set()
                    break

                next_frame_time += interval
                sleep_time = next_frame_time - time.monotonic()
                if sleep_time > 0:
                    time.sleep(sleep_time)
                else:
                    next_frame_time = time.monotonic()
        except Exception as e:
            logger_mp.error(f"Failed to publish frame from {camera_topic_name} camera.")
            stop_event.set()

    def _clean_up(self):
        self._responser.stop()
        for t in self._publisher_threads:
            if t.is_alive():
                t.join(timeout=1.0)
        self._publisher_threads.clear()
        
        for cam in self._cameras.values():
            if cam:
                cam.release()
        self._publisher_manager.close()
        logger_mp.info("[Image Server] Shutdown complete.")

    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    def start(self):
        for camera_topic_name, camera in self._cameras.items():
            t = threading.Thread(target=self._image_pub, args=(camera_topic_name, camera, self._stop_event), daemon=True)
            t.start()
            self._publisher_threads.append(t)

    def wait(self):
        self._stop_event.wait()
        self._clean_up()

    def stop(self):
        self._stop_event.set()

# ========================================================
# utility functions
# ========================================================
def signal_handler(server, signum, frame):
    logger_mp.info(f"[Image Server] Received signal {signum}, initiating graceful shutdown...")
    server.stop()


if __name__ == "__main__":
    logger_mp.info(
        "\n====================== Image Server Startup Guide ======================\n"
        "Please first read this repo's README.md to learn how to configure and use the teleimager.\n"
        "To discover connected cameras, run the following command:\n"
        "\n"
        "    sudo $(which python) image_server.py --cf\n"
        "\n"
        "The '--cf' flag means 'camera find'.\n"
        "This will list all detected cameras and their details (video paths, serial numbers and physical path etc.).\n"
        "Use that information to fill in your 'cam_config.yaml' file.\n"
        "Once configured, you can start the image server with:\n"
        "\n"
        "    sudo $(which python) image_server.py\n"
        "\n"
        "Note:\n"
        " - If you have RealSense cameras, add the '--rs' flag to enable RealSense support.\n"
        " - Make sure you have proper permissions to access the camera devices (e.g., run with sudo or set udev rules).\n"
        "=========================================================================="
    )

    # command line args
    parser = argparse.ArgumentParser()
    parser.add_argument('--cf', action = 'store_true', help = 'Enable camera found mode, print all connected cameras info')
    parser.add_argument('--rs', action = 'store_true', help = 'Enable RealSense camera mode. Otherwise only find UVC/OpenCV cameras.')
    args = parser.parse_args()

    # if enable camera finder mode, just print cameras info and exit
    if args.cf:
        CameraFinder(realsense_enable=args.rs, verbose=True)
        exit(0)

    # Load config file, start image server
    try:
        with open(CONFIG_PATH, "r") as f:
            cam_config = yaml.safe_load(f)
    except Exception as e:
        logger_mp.error(f"Failed to load configuration file at {CONFIG_PATH}: {e}")
        exit(1)

    # start image server
    server = ImageServer(cam_config, realsense_enable=args.rs, camera_finder_verbose=False)
    server.start()

    # graceful shutdown handling
    signal.signal(signal.SIGINT, functools.partial(signal_handler, server))
    signal.signal(signal.SIGTERM, functools.partial(signal_handler, server))

    logger_mp.info("[Image Server] Running... Press Ctrl+C to exit.")
    server.wait()
