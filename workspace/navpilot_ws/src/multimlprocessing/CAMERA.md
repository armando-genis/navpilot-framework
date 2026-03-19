colcon build --packages-select multicamera_processing
colcon build --packages-select pointcloud_rotation
colcon build --packages-select lidar_camera_matcher
colcon build --packages-select multicamera_detection
colcon build --packages-select path_processing

colcon build --packages-select multicamera_detection_tensort


colcon build --packages-select multicamera_detection_tensort

ros2 launch multicamera_processing multiprocessing.launch.py

ros2 launch multicamera_detection multiprocessing.launch.py

sudo apt install libyaml-cpp-dev

ros2 launch obsbot_multicamera.launch.py

ros2 launch obsbot_multicamera.launch.py

<!-- lidar -->

ros2 launch sensors_launch velodyne-VLP32C-launch.py

ros2 launch pointcloud_rotation lidar_rotation.launch.py

ros2 launch lidar_camera_matcher sync.launch.py

ros2 launch path_processing multiprocessing.launch.py

find /dev -maxdepth 1 -name "video\*"

```bash
# 1) Turn OFF continuous autofocus
v4l2-ctl -d /dev/video0 -c focus_automatic_continuous=0

# 2) Set a fixed manual focus value (range: 0-100)
v4l2-ctl -d /dev/video0 -c focus_absolute=10

v4l2-ctl -d /dev/video2 -c focus_automatic_continuous=0

# 2) Set a fixed manual focus value (range: 0-100)
v4l2-ctl -d /dev/video2 -c focus_absolute=10
```
