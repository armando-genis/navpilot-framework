# Camera Modules Documentation

## OBSBOT Meet SE Camera

This documentation covers the setup and usage of the OBSBOT Meet SE Full HD webcam with ROS 2.

**Camera Specifications:** [OBSBOT Meet SE Specs](https://www.obsbot.com/es/obsbot-meet-se-full-hd-webcam/specs)

### Camera Overview

The OBSBOT Meet SE exposes two V4L2 video nodes:

- `/dev/video0` - Main video stream
- `/dev/video1` - Secondary video stream

**Supported Formats:**

- `/dev/video0` supports MJPG at 1920×1080 up to 100 fps (also supports 60/30 fps and other rates)
- 720p resolution up to 150 fps
- Low-resolution YUYV mode (for compatibility)

---

## 1. Installation

### Prerequisites (Ubuntu/Debian)

Install required GStreamer packages:

```bash
sudo apt update
sudo apt install -y \
  gstreamer1.0-tools \
  gstreamer1.0-libav \
  gstreamer1.0-plugins-base \
  gstreamer1.0-plugins-good \
  gstreamer1.0-plugins-bad \
  gstreamer1.0-plugins-ugly
```

Install V4L2 utilities:

```bash
sudo apt update
sudo apt install -y v4l-utils
```

Install OpenCV (system package):

```bash
pip3 uninstall -y opencv-python opencv-contrib-python opencv-python-headless
sudo apt update
sudo apt install -y python3-opencv
```

---

## 2. Device Detection

List available video devices:

```bash
find /dev -maxdepth 1 -name "video*"
```

---

## 3. Camera Verification

Verify the OBSBOT camera is recognized as V4L2 device nodes:

```bash
ls -l /dev/video0 /dev/video1
v4l2-ctl --list-devices
v4l2-ctl -d /dev/video0 --list-formats-ext
```

---

## 4. Quick Test

Test both video nodes to validate functionality:

**Test /dev/video0:**

```bash
gst-launch-1.0 v4l2src device=/dev/video0 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegdec ! videoconvert ! autovideosink
```

**Test /dev/video1:**

```bash
gst-launch-1.0 v4l2src device=/dev/video1 ! image/jpeg,width=1920,height=1080,framerate=30/1 ! jpegdec ! videoconvert ! autovideosink
```

---

## v4l2_camera_node

The `v4l2_camera_node` interfaces with standard V4L2 devices and

publishes images as `sensor_msgs/Image` messages.

### Published Topics

- `/image_raw` – `sensor_msgs/Image`
  The image.

---

## Parameters

- `video_device` – `string`, default: `"/dev/video0"`
  The device the camera is on.

---

- `pixel_format` – `string`, default: `"YUYV"`
  The pixel format to request from the camera. Must be a valid four
  character '[FOURCC](http://fourcc.org/)' code supported by V4L2
  and by your camera. The node outputs the available formats
  supported by your camera when started.
  **Supported formats:**
  - `"YUYV"` – YUV 4:2:2 packed (uncompressed)
  - `"UYVY"` – YUV 4:2:2 packed (uncompressed, CUDA-only)
  - `"GREY"` – Monochrome 8-bit
  - `"MJPG"` – Motion-JPEG (compressed)
    **Notes on MJPG:**
  - MJPG is required for high resolutions on many USB cameras
    (e.g. 1920×1080 on OBSBOT Meet SE).
  - MJPG frames are received as compressed JPEG byte streams and
    decoded internally before publication.

---

- `output_encoding` – `string`, default: `"rgb8"`
  The encoding used for the published image.
  **Supported encodings:**
  - `"rgb8"`
  - `"yuv422"`
  - `"mono8"`
    **MJPG behavior:**
  - When `pixel_format` is `"MJPG"` and `output_encoding` is `"rgb8"`,
    MJPG frames are decoded using a JPEG decoder and published as
    RGB images.
  - MJPG frames are **not** published as raw JPEG; decoding always
    occurs before publication.

---

- `image_size` – `integer_array`, default: `[640, 480]`
  Width and height of the image.
  **Important:**
  - For uncompressed formats (`YUYV`, `UYVY`, `GREY`), only resolutions
    supported directly by the camera are valid.
  - For compressed formats (`MJPG`), higher resolutions (e.g. 1280×720,
    1920×1080) are typically available.

---

- `time_per_frame` – `integer_array`, default: current device setting
  The time between two successive frames. The expected value is a
  ratio defined by an array of 2 integers. For instance, `[1, 30]`
  sets a frame rate of 30 Hz.
  If the requested period is not supported, the driver may select
  the closest supported value.

---

- `use_v4l2_buffer_timestamps` – `bool`, default: `true`
  Flag to determine image timestamp behavior. When `true`, images are
  timestamped using the V4L2 buffer timestamps. When `false`, system
  time is used when the image buffer is read.

---

- `timestamp_offset` – `int64_t`, default: `0`
  Offset (in nanoseconds) added to the image timestamp. This is useful
  for correcting pipeline latency when synchronizing with other sensors.

---

## Camera Control Parameters

Camera controls such as brightness, contrast, exposure, white balance, etc. are automatically exposed as ROS parameters.The driver enumerates all V4L2 controls and creates a parameter for each. The parameter name is derived from the control name reported by the camera driver, converted to lowercase, with commas removed and spaces replaced by underscores.

Examples:

- `Brightness` → `brightness`
- `White Balance, Automatic` → `white_balance_automatic`

---

## Implementation Notes (Important)

- **Raw formats (`YUYV`, `UYVY`, `GREY`)**
  - Image buffers are copied using the full frame size
    (`bytes_per_line × height`).
  - This guarantees safe row-based access during color conversion.
- **Compressed format (`MJPG`)**
  - Image buffers are copied using the actual payload size
    (`buf.bytesused`).
  - Frames are decoded internally before publication.
  - This prevents invalid memory reads and JPEG decode errors.

## Compressed Transport

By default `image_transport` only supports raw transfer, plugins are
required to enable compression. Standard ones are available in the
[`image_transport_plugins`](https://github.com/ros-perception/image_transport_plugins)
repository. These depend on the OpenCV facilities provided by the
`vision_opencv` repository. You can clone these into your workspace to
get these:

    cd path/to/workspace
    git clone https://github.com/ros-perception/vision_opencv.git --branch ros2 src/vision_opencv
    git clone https://github.com/ros-perception/image_transport_plugins.git --branch ros2 src/image_transport_plugins

---

## Building from Source

This code has been modified to support MJPG format. The original repository is:

```bash
git clone https://github.com/tier4/ros2_v4l2_camera
```

### Build Instructions

To build the modified `v4l2_camera` package, use the following command:

```bash
colcon build --packages-select v4l2_singlecamera --cmake-clean-cache --cmake-args -DENABLE_CUDA=OFF

colcon build --packages-select v4l2_Multicamera --cmake-clean-cache --cmake-args -DENABLE_CUDA=OFF

```

**Note:** The `-DENABLE_CUDA=OFF` flag disables CUDA support. Remove this flag if you need CUDA acceleration for image processing.
