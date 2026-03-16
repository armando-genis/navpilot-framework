ros2 launch pointcloud_rotation lidar_rotation.launch.py

ros2 launch pointcloud_clustering pointcloud_clustering.launch.py

ros2 launch pointcloud_clustering_KDTree pointcloud_clustering.launch.py

colcon build --packages-select pointcloud_clustering
colcon build --packages-select pointcloud_rotation
colcon build --packages-select pointcloud_clustering_KDTree
colcon build --packages-select coarse_to_fine_localization
colcon build --packages-select lidar_localization_ros2
colcon build --packages-select localization_sound_alert

ros2 launch /workspace/navpilot_ws/src/localization_modules/launch/localization.launch.py
ros2 launch /workspace/navpilot_ws/src/localization_modules/launch_coarse_to_fine/localization.launch.py

colcon build --packages-select path_planning_dynamic

ros2 launch /workspace/navpilot_ws/src/path_planning/launch/planning_obstacles.launch.py

rviz2 -d /workspace/navpilot_ws/src/path_planning/path_planning_viz.rviz

colcon build --packages-select waypoints_simple_creator

ros2 launch waypoints_simple_creator waypoints.launch.py

# lidar modules

colcon build --packages-select pointcloud_clustering
colcon build --packages-select pointcloud_clustering_KDTree
colcon build --packages-select pointcloud_rotation

# camera modules

colcon build --packages-select v4l2_Multicamera --cmake-clean-cache --cmake-args -DENABLE_CUDA=OFF

#processing
colcon build --packages-select multicamera_processing
colcon build --packages-select lidar_camera_matcher
colcon build --packages-select multicamera_detection
colcon build --packages-select path_processing
