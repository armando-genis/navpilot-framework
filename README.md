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



## TODO:
- add this to the setup.sh
git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
- to build it:

colcon build --packages-select mrt_cmake_modules
colcon build --packages-select polygon_msgs 
colcon build --packages-select polygon_rviz_plugins
colcon build --packages-select polygon_demos 
colcon build --packages-select polygon_utils 
colcon build --packages-select traffic_information_msgs
source install/setup.bash
colcon build --packages-select lanelet2_core
colcon build --packages-select lanelet2_maps
colcon build --packages-select lanelet2_io
colcon build --packages-select lanelet2_projection
colcon build --packages-select lanelet2_traffic_rules
colcon build --packages-select lanelet2_routing
colcon build --packages-select lanelet2_validation
colcon build --packages-select lanelet2_python
colcon build --packages-select lanelet2_examples

colcon build --packages-select map_visualizer







