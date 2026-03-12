# navpilot-framework

███╗░░██╗░█████╗░██╗░░░██╗██████╗░██╗██╗░░░░░░█████╗░████████╗
████╗░██║██╔══██╗██║░░░██║██╔══██╗██║██║░░░░░██╔══██╗╚══██╔══╝
██╔██╗██║███████║╚██╗░██╔╝██████╔╝██║██║░░░░░██║░░██║░░░██║░░░
██║╚████║██╔══██║░╚████╔╝░██╔═══╝░██║██║░░░░░██║░░██║░░░██║░░░
██║░╚███║██║░░██║░░╚██╔╝░░██║░░░░░██║███████╗╚█████╔╝░░░██║░░░
╚═╝░░╚══╝╚═╝░░╚═╝░░░╚═╝░░░╚═╝░░░░░╚═╝╚══════╝░╚════╝░░░░╚═╝░░░

### → Step 1: Build the Docker Image

```bash
sudo docker compose build
```

### → Step 2: Start the Container

The `-d` flag ensures the container runs in detached mode (in the background).

```bash
sudo docker compose up -d
```

### → Step 3: Access the Container

Use either `bash` or `zsh` to start a shell inside the container.

```bash
docker exec -it navpilot_container zsh
```

### → Step 4: 📦 Install GitHub repositories & dependencies (colcon build)

The repository is mounted inside the container at **`/workspace`**.

If you are not already in `/workspace`, run:

```bash
cd /workspace
```

Run the setup/build script:

```bash
chmod +x setup_file.sh
./setup_file.sh
```

setup network:
sudo ip addr add 192.168.1.100/24 dev enp2s0

sudo iptables -I INPUT -p udp --dport 2368 -j ACCEPT
sudo sysctl -w net.ipv4.conf.enp2s0.rp_filter=0

## → Sensor Launchers

Launch individual or combined sensor configurations as needed:

- For LiDAR only:

```bash
ros2 launch sensors_launch velodyne-VLP32C-launch.py
```

- For IMU only:

```bash
ros2 launch sensors_launch vectornav.launch.py
```

- For LiDAR and IMU combined:

```bash
ros2 launch sensors_launch lidar_imu.launch.py
```

## → 🗺️ Launchers for mapping

Run the following commands to initialize mapping processes:

```bash
ros2 launch /workspace/navpilot_ws/src/mapping_modules/launcher/mapping.launch.py
```

## → 🌏 Launchers for localization

Run the following command to launch localization:

```bash
ros2 launch /workspace/navpilot_ws/src/localization_modules/launch/localization.launch.py
```

## → 🛣️ Launchers for path plannning

```bash
ros2 launch /workspace/navpilot_ws/src/path_planning/launch/planning_obstacles.launch.py
```

## → 📢 Code Modifications Before colcon build

Before building the package, make the following changes to the file lidar_localization_component.cpp located in the src directory of lidar_localization_ros2. These adjustments will modify the default subscriber topics to match the correct topics of the car.
Navigate to:

- `workspace/navpilot_ws/src/localization_modules/lidar_localization_ros2/src/lidar_localization_component.cpp`

Then locate `initializePubSub()` and update the `cloud_sub_` / `imu_sub_` topics to match your platform at the line 234 and 238.

```bash
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_rotated", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "vectornav/imu", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));
```

## → Intrinsic LiDAR–camera calibration

- **Docs (start here)**: for LiDAR–camera calibration, please refer to:

  - `workspace/navpilot_ws/src/lidar_camera_calib/README_CALIBRATE.md`

- **Calibration board output files** (generated in the current directory):

  - **`calibration_board.png`** - PNG image of the board (for preview)
  - **`calibration_board.pdf`** - PDF file ready for printing (1:1 scale, 1mm = 1px)
  - **`calibration_board.svg`** - Vectorized SVG format (for high-quality printing)
  - **`calibration_board_config.yaml`** - Configuration file with board specifications (used by the calibration framework)

Then run the `multisensor_calibration` package.

> [!IMPORTANT]
> Read that README carefully before building/running `multisensor_calibration`, to compile it correctly and avoid dependency/build issues.

## → Cameras setup

- **Docs (recommended)**: for multi-camera stack acquisition, please read:

  - `workspace/navpilot_ws/src/camera_modules/CAMERA.md`

- **What it contains**:

  - calibration files
  - a Python example
  - the `v4l2_camera` package with added MJPG support for the OBSBOT Meet SE cameras

- **Example: encode V4L2 in Python / ROS 2**:

```bash
python3 /workspace/navpilot_ws/src/camera_modules/GStreamerMultiCameraNode.py
```

- **Launch the v4l2_camera package for data streaming**:

```bash
ros2 launch workspace/navpilot_ws/src/camera_modules/obsbot_camera.launch.py
```

## → BIN files (from PointCloud)

To create BIN files from point clouds, a script is provided:

```bash
python3 /workspace/navpilot_ws/src/pc2_to_bin/pc2_to_bin_timer.py
```

## record rosbag mcap

```bash
ros2 bag record --storage mcap --all --output vanttec_sdv_localization_20250623
```

## play a ros bag

```bash
rviz2 -d /workspace/navpilot_ws/src/localization_modules/launch/localization.rviz

ros2 bag play vanttec_sdv_localization_20250623 -s mcap
```
