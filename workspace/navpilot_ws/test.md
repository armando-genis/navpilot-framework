ros2 launch pointcloud_rotation lidar_rotation.launch.py

ros2 launch pointcloud_clustering pointcloud_clustering.launch.py

ros2 launch pointcloud_clustering_KDTree pointcloud_clustering.launch.py

colcon build --packages-select pointcloud_clustering
colcon build --packages-select pointcloud_rotation
colcon build --packages-select pointcloud_clustering_KDTree

colcon build --packages-select coarse_to_fine_localization

ros2 launch /workspace/navpilot_ws/src/localization_modules/launch/localization.launch.py
ros2 launch /workspace/navpilot_ws/src/localization_modules/launch_coarse_to_fine/localization.launch.py
