# navpilot-framework

### → Step 1:   Build the Docker Image

```bash
sudo docker compose build
```

### → Step 2:  Start the Container

The `-d` flag ensures the container runs in detached mode (in the background).

```bash
sudo docker compose up -d
```

### → Step 2: Access the Container

Use either `bash` or `zsh` to start a shell inside the container.

```bash
docker exec -it navpilot_container zsh
```

### → Step 3: 📦 Install github repository & dependencies

```bash
chmod +x setup_file.sh
./setup_file.sh
```

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

## → 🌏 Launchers for mapping
Run the following commands to initialize mapping processes:
```bash
ros2 launch /workspace/navpilot_ws/src/mapping_modules/launcher/mapping.launch.py
```

## → 🌏 Launchers for localization
Run the following commands to initialize mapping processes:
```bash
ros2 launch /workspace/navpilot_ws/src/localization_modules/launch/localization.launch.py
```

## → 📢 Code Modifications Before colcon build
Before building the package, make the following changes to the file lidar_localization_component.cpp located in the src directory of lidar_localization_ros2. These adjustments will modify the default subscriber topics to match the correct topics of the car. 
Navigate to `lidar_localization_ros2/src/lidar_localization_component.cpp` and change the lines 234 and 238 for this ones:

```bash
  cloud_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "points_rotated", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::cloudReceived, this, std::placeholders::_1));

  imu_sub_ = create_subscription<sensor_msgs::msg::Imu>(
      "vectornav/imu", rclcpp::SensorDataQoS(),
      std::bind(&PCLLocalization::imuReceived, this, std::placeholders::_1));
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

