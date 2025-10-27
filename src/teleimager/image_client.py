import cv2
import time
from . import zmq_msg
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import logging_mp
logger_mp = logging_mp.get_logger(__name__, level=logging_mp.INFO)

class ImageClient:
    def __init__(self, host="192.168.123.164", request_port=60000):
        """
        Args:
            server_address:   IP address of image host server
            requset_port:     Port for request camera configuration
        """
        self._host = host
        self._request_port = request_port
        self._aspect_ratio_threshold = 2.0 # If the aspect ratio exceeds this value, it is considered binocular. ohterwise, monocular.

        # requester and subscriber setup
        self._requester  = zmq_msg.Requester(self._host, self._request_port)
        self._subscriber_manager = zmq_msg.SubscriberManager.get_instance()

        # request camera configuration
        self.camconfig = self._requester.request()
        logger_mp.debug(f"Received camera config: {self.camconfig}")

        # parse camera configuration
        # default Head camera
        self._enable_head_zmq = False
        self._head_zmq_port = 55555
        self._enable_head_webrtc = False
        self._head_webrtc_port = 60001
        self._head_image_shape = (480, 1280)
        self._head_server_fps = 30.0
        self._binocular = None
        # default Left wrist camera
        self._enable_left_wrist_zmq = False
        self._left_wrist_zmq_port = 55556
        self._enable_left_wrist_webrtc = False
        self._left_wrist_webrtc_port = 60002
        self._left_wrist_image_shape = (480, 640)
        self._left_wrist_server_fps = 30.0
        # default Right wrist camera
        self._enable_right_wrist_zmq = False
        self._right_wrist_zmq_port = 55557
        self._enable_right_wrist_webrtc = False
        self._right_wrist_webrtc_port = 60003
        self._right_wrist_image_shape = (480, 640)
        self._right_wrist_server_fps = 30.0

        if self.camconfig is not None:
            # head camera
            self._enable_head_zmq = self.camconfig['head_camera']['enable_zmq']
            if self._enable_head_zmq:
                self._head_zmq_port = self.camconfig['head_camera']['zmq_port']
                self._subscriber_manager.subscribe(self._host, self._head_zmq_port)
            
            self._enable_head_webrtc = self.camconfig['head_camera']['enable_webrtc']
            self._head_webrtc_port = self.camconfig['head_camera']['webrtc_port']
            self._head_webrtc_url = f"https://{self._host}:{self._head_webrtc_port}/offer"
            self._head_image_shape = self.camconfig['head_camera']['image_shape']
            self._head_cam_server_fps = self.camconfig['head_camera']['fps'] # fps set on the server side
            self._binocular = self.camconfig['head_camera']['image_shape'][1] / self.camconfig['head_camera']['image_shape'][0] > self._aspect_ratio_threshold
            
            # left wrist camera
            self._enable_left_wrist_zmq = self.camconfig['left_wrist_camera']['enable_zmq']
            if self._enable_left_wrist_zmq:
                self._left_wrist_zmq_port = self.camconfig['left_wrist_camera']['zmq_port']
                self._subscriber_manager.subscribe(self._host, self._left_wrist_zmq_port)
            
            self._enable_left_wrist_webrtc = self.camconfig['left_wrist_camera']['enable_webrtc']
            self._left_wrist_webrtc_port = self.camconfig['left_wrist_camera']['webrtc_port']
            self._left_wrist_webrtc_url = f"https://{self._host}:{self._left_wrist_webrtc_port}/offer"
            self._left_wrist_image_shape = self.camconfig['left_wrist_camera']['image_shape']
            self._left_wrist_server_fps = self.camconfig['left_wrist_camera']['fps'] # fps set on the server side

            # right wrist camera
            self._enable_right_wrist_zmq = self.camconfig['right_wrist_camera']['enable_zmq']
            if self._enable_right_wrist_zmq:
                self._right_wrist_zmq_port = self.camconfig['right_wrist_camera']['zmq_port']
                self._subscriber_manager.subscribe(self._host, self._right_wrist_zmq_port)

            self._enable_right_wrist_webrtc = self.camconfig['right_wrist_camera']['enable_webrtc']
            self._right_wrist_webrtc_port = self.camconfig['right_wrist_camera']['webrtc_port']
            self._right_wrist_webrtc_url = f"https://{self._host}:{self._right_wrist_webrtc_port}/offer"
            self._right_wrist_image_shape = self.camconfig['right_wrist_camera']['image_shape']
            self._right_wrist_server_fps = self.camconfig['right_wrist_camera']['fps'] # fps set on the server side
        else:
            logger_mp.error("Failed to get camera configuration from server. Please check image server whether it is running.")

        if not self._enable_head_zmq and not self._enable_head_webrtc:
            logger_mp.warning("[Image Client] NOTICE! Head camera is not enabled on both ZMQ and WebRTC.")
        # Wait for the subscriber to initialize
        time.sleep(0.005)
    
    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    # Head camera
    def head_is_binocular(self) -> bool:
        return self._binocular

    def enable_head_zmq(self) -> bool:
        return self._enable_head_zmq
    
    def get_head_shape(self) -> tuple:
        return self._head_image_shape

    def get_head_frame(self):
        """Get the latest head camera frame, and the current receiving FPS."""
        if self._enable_head_zmq:
            return self._subscriber_manager.subscribe(self._host, self._head_zmq_port)

    def enable_head_webrtc(self) -> bool:
        return self._enable_head_webrtc

    def head_webrtc_url(self) -> str:
        return self._head_webrtc_url
    
    # Left wrist camera
    def enable_left_wrist_zmq(self) -> bool:
        return self._enable_left_wrist_zmq

    def get_left_wrist_shape(self) -> tuple:
        return self._left_wrist_image_shape

    def get_left_wrist_frame(self):
        """Get the latest left wrist camera frame, and the current receiving FPS."""
        if self._enable_left_wrist_zmq:
            return self._subscriber_manager.subscribe(self._host, self._left_wrist_zmq_port)
        else:
            logger_mp.warning("Left wrist camera is not enabled.")
            return None, 0.0
    
    def enable_left_wrist_webrtc(self) -> bool:
        return self._enable_left_wrist_webrtc

    def left_wrist_webrtc_url(self) -> str:
        return self._left_wrist_webrtc_url

    # Right wrist camera
    def enable_right_wrist_zmq(self) -> bool:
        return self._enable_right_wrist_zmq

    def get_right_wrist_shape(self) -> tuple:
        return self._right_wrist_image_shape

    def get_right_wrist_frame(self):
        """Get the latest right wrist camera frame, and the current receiving FPS."""
        if self._enable_right_wrist_zmq:
            return self._subscriber_manager.subscribe(self._host, self._right_wrist_zmq_port)
        else:
            logger_mp.warning("Right wrist camera is not enabled.")
            return None, 0.0
    
    def enable_right_wrist_webrtc(self) -> bool:
        return self._enable_right_wrist_webrtc   

    def right_wrist_webrtc_url(self) -> str:
        return self._right_wrist_webrtc_url

    def close(self):
        self._subscriber_manager.close()
        logger_mp.info("Image client has been closed.")

if __name__ == "__main__":
    # Example usage with three camera streams
    client = ImageClient(host='127.0.0.1')  # Change to '127.0.0.1' for local test
    
    running = True
    while running:
        if client.enable_head_zmq():
            head_img, head_fps = client.get_head_frame()
            head_shape = client.get_head_shape()
            binocular = client.head_is_binocular()
            logger_mp.info(f"Head Camera FPS: {head_fps:.2f}")
            logger_mp.debug(f"Head Camera Shape: {head_shape}")
            logger_mp.debug(f"Head Camera Binocular: {binocular}")
            if head_img is not None:
                cv2.imshow("Head Camera", head_img)

        if client.enable_left_wrist_zmq():
            left_wrist_img, left_wrist_fps = client.get_left_wrist_frame()
            left_wrist_shape = client.get_left_wrist_shape()
            logger_mp.info(f"Left Wrist Camera FPS: {left_wrist_fps:.2f}")
            logger_mp.debug(f"Left Wrist Camera Shape: {left_wrist_shape}")
            if left_wrist_img is not None:
                cv2.imshow("Left Wrist Camera", left_wrist_img)

        if client.enable_right_wrist_zmq():
            right_wrist_img, right_wrist_fps = client.get_right_wrist_frame()
            right_wrist_shape = client.get_right_wrist_shape()
            logger_mp.info(f"Right Wrist Camera FPS: {right_wrist_fps:.2f}")
            logger_mp.debug(f"Right Wrist Camera Shape: {right_wrist_shape}")
            if right_wrist_img is not None:
                cv2.imshow("Right Wrist Camera", right_wrist_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            logger_mp.info("Exiting image client on user request.")
            running = False
            # clean up
            client.close()
            cv2.destroyAllWindows()
        # Small delay to prevent excessive CPU usage
        time.sleep(0.002)
        

        