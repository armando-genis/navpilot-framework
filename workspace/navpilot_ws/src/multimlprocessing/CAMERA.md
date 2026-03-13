colcon build --packages-select multicamera_processing
colcon build --packages-select pointcloud_rotation

ros2 launch multicamera_processing multiprocessing.launch.py

sudo apt install libyaml-cpp-dev

ros2 launch obsbot_multicamera.launch.py

<!-- lidar -->

ros2 launch pointcloud_rotation lidar_rotation.launch.py
