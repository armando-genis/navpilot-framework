source install/setup.bash
colcon build --packages-select camera_lidar_sync

ros2 launch camera_lidar_sync sync.launch.py
