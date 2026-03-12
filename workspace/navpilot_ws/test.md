ros2 launch pointcloud_rotation lidar_rotation.launch.py

ros2 launch pointcloud_clustering pointcloud_clustering.launch.py

ros2 launch pointcloud_clustering_KDTree pointcloud_clustering.launch.py

colcon build --packages-select pointcloud_clustering
colcon build --packages-select pointcloud_rotation
