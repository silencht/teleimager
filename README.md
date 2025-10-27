# Tele Imager

## 1. Image Server

This module provides an image server that captures video streams from multiple cameras (UVC、OpenCV and RealSense) and publishes the frames over a network using ZeroMQ. It is designed to support teleoperation applications where real-time video streaming is essential. 

All user-callable functions are listed below the `# public api` comment in the code.

### 1.0 ✨ Features

- 📸 Supports multiple UVC, OpenCV and Intel RealSense cameras.
- 📢 Publishes video frames using ZeroMQ PUB-SUB pattern.
- 🚧 (TODO)  LocalHost shared memory mode for low-latency frame access.
- 💬 Response image configuration via ZeroMQ REQ-REP pattern.
- 🆔 Multi Camera identifiers (choose one or more): physical path, serial number and video device path.
- ⚙️ Configurable frame resolution and frame rate.
- 🚀 Efficient frame handling using a triple ring buffer.

### 1.1 📥 Environment Setup

You can set up TeleImager using either a Conda environment or the system (base) environment.
Both methods work — choose whichever fits your workflow better.

#### 1.1.1 Conda Environment
1. Clone the repository:
    ```bash
    git clone https://github.com/silencht/teleimager.git
    cd teleimager
    ```
2. Create and activate a Python virtual environment (optional but recommended):
    ```bash
    conda create -n teleimager python=3.10 -y
    conda activate teleimager
    ```
3. Install the required packages:
    ```bash
    sudo apt install -y libusb-1.0-0-dev libturbojpeg-dev 
    pip install -e .
    ```
#### 1.1.2 Base Environment
1. Clone the repository:
    ```bash
    git clone https://github.com/silencht/teleimager.git
    cd teleimager
    ```
2. Install the required packages:
   ```bash
    sudo apt install -y libusb-1.0-0-dev libturbojpeg-dev
    sudo pip install --upgrade pip setuptools wheel
    pip install .
   ```

### 1.2 🔍 Finding connected cameras

To discover connected cameras, run the following command in the terminal. 
`sudo $(which python)` is used to ensure the script has the necessary permissions to access camera devices. And `--cf` means "camera find". 
```bash
(teleimager) unitree@unitree:~/image_server$ sudo $(which python) image_server.py --cf
```

You should see output similar to the following, which lists all detected UVC cameras and their details:

```bash
(teleimager) unitree@unitree:~/xr_teleoperate/teleop/image_server$ sudo $(which python) image_server.py --cf
10:24:35:849900 INFO     ======================= Camera Discovery Start ================================== image_server.py:216
10:24:35:851008 INFO     Found video devices: ['/dev/video0', '/dev/video1', '/dev/video2', '/dev/video3', image_server.py:217
                         '/dev/video4', '/dev/video5']                                                                        
10:24:35:852089 INFO     Found RGB video devices: ['/dev/video0', '/dev/video2', '/dev/video4']            image_server.py:218
10:24:35:852280 INFO     ------------------------- UVC Camera 1 ------------------------------------       image_server.py:227
10:24:35:852575 INFO     video_path    : /dev/video0                                                       image_server.py:228
10:24:35:852759 INFO     video_id      : 0                                                                 image_server.py:229
10:24:35:852844 INFO     serial_number : 200901010002                                                      image_server.py:230
10:24:35:852919 INFO     physical_path : /sys/devices/pci0000:00/0000:00:14.0/usb1/1-5/1-5:1.0             image_server.py:231
10:24:35:852989 INFO     extra_info:                                                                       image_server.py:232
10:24:35:853062 INFO         name: USB HDR Camera                                                          image_server.py:239
10:24:35:853133 INFO         manufacturer: Generic                                                         image_server.py:239
10:24:35:853198 INFO         serialNumber: 200901010002                                                    image_server.py:239
10:24:35:853261 INFO         idProduct: 8272                                                               image_server.py:239
10:24:35:853336 INFO         idVendor: 7749                                                                image_server.py:239
10:24:35:853399 INFO         device_address: 4                                                             image_server.py:239
10:24:35:853735 INFO         bus_number: 1                                                                 image_server.py:239
10:24:35:853829 INFO         uid: 1:4                                                                      image_server.py:239
...
10:24:36:033234 INFO         format: 480x640@30 MJPG                                                       image_server.py:243
10:24:36:033249 INFO         format: 480x640@60 MJPG                                                       image_server.py:243
...
10:24:36:034519 INFO     ------------------------- UVC Camera 2 ------------------------------------       image_server.py:227
10:24:36:034551 INFO     video_path    : /dev/video2                                                       image_server.py:228
10:24:36:034567 INFO     video_id      : 2                                                                 image_server.py:229
10:24:36:034582 INFO     serial_number : 01.00.00                                                          image_server.py:230
10:24:36:034595 INFO     physical_path : /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.1/1-11.1:1.0  image_server.py:231
10:24:36:034608 INFO     extra_info:                                                                       image_server.py:232
10:24:36:034622 INFO         name: Cherry Dual Camera                                                      image_server.py:239
10:24:36:034635 INFO         manufacturer: DECXIN                                                          image_server.py:239
10:24:36:034647 INFO         serialNumber: 01.00.00                                                        image_server.py:239
10:24:36:034658 INFO         idProduct: 11599                                                              image_server.py:239
10:24:36:034670 INFO         idVendor: 7119                                                                image_server.py:239
10:24:36:034683 INFO         device_address: 9                                                             image_server.py:239
10:24:36:034695 INFO         bus_number: 1                                                                 image_server.py:239
10:24:36:034710 INFO         uid: 1:9                                                                      image_server.py:239
...
10:24:36:435928 INFO         format: 480x1280@10 MJPG                                                      image_server.py:243
10:24:36:435988 INFO         format: 480x1280@15 MJPG                                                      image_server.py:243
10:24:36:436047 INFO         format: 480x1280@20 MJPG                                                      image_server.py:243
10:24:36:436108 INFO         format: 480x1280@25 MJPG                                                      image_server.py:243
10:24:36:436168 INFO         format: 480x1280@30 MJPG                                                      image_server.py:243
10:24:36:436227 INFO         format: 480x1280@60 MJPG                                                      image_server.py:243
10:24:36:436286 INFO         format: 480x1280@120 MJPG                                                     image_server.py:243
...
10:24:36:524038 INFO     ------------------------- UVC Camera 3 ------------------------------------       image_server.py:227
10:24:36:524203 INFO     video_path    : /dev/video4                                                       image_server.py:228
10:24:36:524282 INFO     video_id      : 4                                                                 image_server.py:229
10:24:36:524345 INFO     serial_number : 200901010001                                                      image_server.py:230
10:24:36:524398 INFO     physical_path : /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.2/1-11.2:1.0  image_server.py:231
10:24:36:524449 INFO     extra_info:                                                                       image_server.py:232
10:24:36:524531 INFO         name: USB HDR Camera                                                          image_server.py:239
10:24:36:524672 INFO         manufacturer: Generic                                                         image_server.py:239
10:24:36:524734 INFO         serialNumber: 200901010001                                                    image_server.py:239
10:24:36:524789 INFO         idProduct: 8272                                                               image_server.py:239
10:24:36:524843 INFO         idVendor: 7749                                                                image_server.py:239
10:24:36:524893 INFO         device_address: 10                                                            image_server.py:239
10:24:36:524942 INFO         bus_number: 1                                                                 image_server.py:239
10:24:36:524989 INFO         uid: 1:10                                                                     image_server.py:239
10:24:36:688311 INFO         format: 240x320@30 MJPG                                                       image_server.py:243
...
10:24:36:689031 INFO         format: 480x640@30 MJPG                                                       image_server.py:243
10:24:36:689089 INFO         format: 480x640@60 MJPG                                                       image_server.py:243
...
10:24:36:714374 INFO     =========================== Camera Discovery End ================================
```

If there exists Intel RealSense cameras, you will also see the RealSense camera discovery results like below:

```bash
11:30:49:303683 INFO     ----------------------- Realsense Cameras ----------------------------------
11:30:49:303699 INFO     RealSense serial numbers: ['141722079879']
11:30:49:303712 INFO     RealSense video paths:
                         ['/dev/video4', '/dev/video5', '/dev/video6',
                          '/dev/video7', '/dev/video8', '/dev/video9']  
11:30:49:303724 INFO     RealSense RGB-like video paths: ['/dev/video6', '/dev/video8']
```

### 1.3 📡 Starting the Image Server
According to the camera discovery results up, you can fill `cam_config.yaml` with the results. Here is an example configuration file,

```yaml
# cam_config.yaml, All parameters are explained in the comments.
# =====================================================
# Head camera configuration
# =====================================================
# camera topic
head_camera:
  # camera configuration

  # Set to true to enable this camera, false to disable
  enable: true

  # Port to publish camera stream, i.e. zmq tcp://*:55555.  image_client.py should connect to the same port
  port : 55555

  # Type of camera:
  #   - "opencv"    → opencv driver
  #   - "realsense" → pyrealsense2 driver
  #   - "uvc"       → pyuvc driver  (Recommended)
  type: uvc

  # Image Format
  # image resolution: [height, width]
  image_shape: [480, 1280]

  # frame per second
  fps: 60

  # Camera identifiers (choose one or more):
  #   - video_id: X        → /dev/videoX  (e.g. 0 → /dev/video0)
  #   - serial_number: Y   → camera's hardware serial (e.g. 141722079879)
  #   - physical_path: Z   → sysfs physical USB path (e.g. /sys/devices/pci0000:00/.../1-11.2:1.0)
  #
  # Identifier priority:
  #   physical_path > serial_number > video_id
  #   if an identifier is not used, set it to null. The system will resolve the camera by priority.
  #
  # Notes:
  #   - type "realsense": supports serial_number only (but a RealSense can also be used as opencv/uvc if desired)
  #   - type "opencv":    supports video_id, serial_number, physical_path
  #   - type "uvc":       supports video_id, serial_number, physical_path

  # - If camera's serial_numbers is unique, you could just use serial_number to identify them. Unfortunately,
  #   some (low-cost) cameras may have the same serial_number. In this case, you could use physical_path to identify them.
  # - So, if you are sure that camera would always be plugged into the same USB port, you could use physical_path.
  #   physical_path is the most robust way to identify a camera, but it is not flexible: 
  #   if you change camera to plug into another USB port, then you need to change the physical_path in the config file.
  # - Finally, video_id is the most fragile way to identify a camera, because it may change when camera plug in order changes.
  #   but if you just want to use video_id, please make sure to set physical_path and serial_number to null!
  #   once you specify either `physical_path` or `serial_number`, the system will no longer fall back to searching by `video_id`.
  #   ——— even if no camera matches the given path/serial.
  video_id: 2                    # according to the discovery result, you could find the UVC Camera2's video_path is /dev/video2
  serial_number: 01.00.00        # you could find the UVC Camera2's serial_number is 01.00.00
  physical_path: null            # you could find the UVC Camera2's physical_path is /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.1/1-11.1:1.0
                                 # for flexibility, we set it to null here, so the system will resolve the camera by serial_number.

# =====================================================
# Left wrist camera configuration
# =====================================================
left_wrist_camera:
  enable: True
  port : 55556
  type: uvc
  image_shape: [480, 640]
  fps: 60
  # initialize UVC Camera3. 
  # first try to use physical_path to find the camera,
  # if not found, then try to use serial_number,
  # if not found, then try to use video_id.
  video_id: 4
  serial_number: 200901010001    # you also could fill realsense serial number here if you are using realsense camera
  physical_path: /sys/devices/pci0000:00/0000:00:14.0/usb1/1-11/1-11.2/1-11.2:1.0 # when you use RealSense camera, physical_path should be null

# =====================================================
# Right wrist camera configuration
# =====================================================
right_wrist_camera:
  enable: True
  port : 55557
  type: uvc
  image_shape: [480, 640]
  fps: 60
  video_id: 4
  serial_number: 200901010002
  physical_path: null
```



## 2. Image Client
This module provides an image client that connects to the image server to receive and display video streams from multiple cameras. It is designed to work with the image server for teleoperation applications.

All user-callable functions are listed below the `# public api` comment in the code.

### 2.1 🎯 Usage
When the image server is running, you can start the image client to receive and display the video streams. Run the following command in another terminal:
```bash
python image_client.py
```
Maybe you need to change the host ip and camera port in `image_client.py` to match the server settings.
For example, if the server is running on Unitree G1 Jetson Nx machine with IP address `192.168.123.164`, you should change the host in `image_client.py` like this:
```python
if __name__ == "__main__":
    # Example usage with three camera streams
    client = ImageClient(host='192.168.123.164')  # Change to the server's IP address
#...
```
> Remember to have the **opencv-python** library installed in your environment.

Then you should see the video streams from the cameras in separate OpenCV windows.