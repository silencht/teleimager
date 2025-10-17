import cv2
import time
import logging_mp
import os
import sys
current_dir = os.path.dirname(os.path.abspath(__file__))
sys.path.append(current_dir)
import messaging
logger_mp = logging_mp.get_logger(__name__, level=logging_mp.DEBUG)

class ImageClient:
    def __init__(self, host="192.168.123.164", head_port=55555, left_wrist_port=55556, right_wrist_port=55557, request_port=60000):
        """
        Args:
            server_address:   IP address of image host server
            head_port:        Port for subscribe head camera
            left_wrist_port:  Port for subscribe left wrist camera
            right_wrist_port: Port for subscribe right wrist camera
            requset_port:     Port for request camera configuration
        """
        self._host = host
        self._head_port = head_port
        self._left_wrist_port = left_wrist_port
        self._right_wrist_port = right_wrist_port
        self._request_port = request_port

        # Subscriber and requester setup
        self._requester  = messaging.Requester(self._host, self._request_port)
        self._subscriber_manager = messaging.SubscriberManager.get_instance()

        # Camera configuration
        self._aspect_ratio_threshold = 2.0 # If the aspect ratio exceeds this value, it is considered binocular. ohterwise, monocular.
        self._binocular = True             # default binocular
        self._has_head_cam = True          # default always enable head camera
        self._head_cam_shape = (480, 1280)
        self._head_cam_server_fps = 30.0

        self._has_left_wrist_cam = False
        self._left_wrist_cam_shape = (480, 640)
        self._left_wrist_cam_server_fps = 30.0

        self._has_right_wrist_cam = False
        self._right_wrist_cam_shape = (480, 640)
        self._right_wrist_cam_server_fps = 30.0
        # request camera configuration
        self.camconfig = self._request_camconfig()
        if self.camconfig is not None:
            # head camera
            self._has_head_cam = self.camconfig['head_camera']['enable']
            if self._has_head_cam:
                self._binocular = self.camconfig['head_camera']['image_shape'][1] / self.camconfig['head_camera']['image_shape'][0] > self._aspect_ratio_threshold
                self._head_cam_shape = self.camconfig['head_camera']['image_shape']
                self._head_cam_server_fps = self.camconfig['head_camera']['fps'] # fps set on the server side
                self._subscriber_manager.subscribe(self._host, self._head_port)
            # left wrist camera
            self._has_left_wrist_cam = self.camconfig['left_wrist_camera']['enable']
            if self._has_left_wrist_cam:
                self._left_wrist_cam_shape = self.camconfig['left_wrist_camera']['image_shape']
                self._left_wrist_cam_server_fps = self.camconfig['left_wrist_camera']['fps'] # fps set on the server side
                self._subscriber_manager.subscribe(self._host, self._left_wrist_port)
            # right wrist camera
            self._has_right_wrist_cam = self.camconfig['right_wrist_camera']['enable']
            if self._has_right_wrist_cam:
                self._right_wrist_cam_shape = self.camconfig['right_wrist_camera']['image_shape']
                self._right_wrist_cam_server_fps = self.camconfig['right_wrist_camera']['fps'] # fps set on the server side
                self._subscriber_manager.subscribe(self._host, self._right_wrist_port)
        else:
            logger_mp.error("Failed to get camera configuration from server. Please check image server whether it is running.")

        # Wait for the subscriber to initialize
        time.sleep(0.005)

    def _request_camconfig(self):
        cam_config = self._requester.request()
        logger_mp.debug(f"Received camera config: {cam_config}")
        return cam_config
    
    # --------------------------------------------------------
    # public api
    # --------------------------------------------------------
    def is_binocular(self):
        return self._binocular

    def has_head_cam(self):
        return self._has_head_cam
    
    def get_head_shape(self):
        return self._head_cam_shape

    def get_head_frame(self):
        """Get the latest head camera frame, and the current receiving FPS."""
        if self._has_head_cam:
            return self._subscriber_manager.subscribe(self._host, self._head_port)

    def has_left_wrist_cam(self):
        return self._has_left_wrist_cam

    def get_left_wrist_frame(self):
        """Get the latest left wrist camera frame, and the current receiving FPS."""
        if self._has_left_wrist_cam:
            return self._subscriber_manager.subscribe(self._host, self._left_wrist_port)
        else:
            logger_mp.warning("Left wrist camera is not enabled.")
            return None, 0.0

    def has_right_wrist_cam(self):
        return self._has_right_wrist_cam

    def get_right_wrist_frame(self):
        """Get the latest right wrist camera frame, and the current receiving FPS."""
        if self._has_right_wrist_cam:
            return self._subscriber_manager.subscribe(self._host, self._right_wrist_port)
        else:
            logger_mp.warning("Right wrist camera is not enabled.")
            return None, 0.0

    def close(self):
        self._subscriber_manager.close()
        logger_mp.info("Image client has been closed.")

if __name__ == "__main__":
    # Example usage with three camera streams
    client = ImageClient(
        host='192.168.123.164',  # Change to '127.0.0.1' for local test
        head_port=55555,
        left_wrist_port=55556,
        right_wrist_port=55557,
        request_port=60000
    )
    
    # client.test()
    running = True
    while running:
        if client.has_head_cam():
            head_img, head_fps = client.get_head_frame()
            head_shape = client.get_head_shape()
            binocular = client.is_binocular()
            logger_mp.info(f"Head Camera FPS: {head_fps:.2f}")
            logger_mp.info(f"Head Camera Shape: {head_shape}")
            logger_mp.info(f"Head Camera Binocular: {binocular}")
            cv2.imshow("Head Camera", head_img)

        if client.has_left_wrist_cam():
            left_wrist_img, left_wrist_fps = client.get_left_wrist_frame()
            cv2.imshow("Left Wrist Camera", left_wrist_img)

        if client.has_right_wrist_cam():
            right_wrist_img, right_wrist_fps = client.get_right_wrist_frame()
            cv2.imshow("Right Wrist Camera", right_wrist_img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            logger_mp.info("Exiting image client on user request.")
            running = False
            # clean up
            client.close()
            cv2.destroyAllWindows()
        # Small delay to prevent excessive CPU usage
        time.sleep(0.002)
        

        