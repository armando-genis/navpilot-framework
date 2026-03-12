#!/bin/bash

# Go to the source folder
cd /workspace/navpilot_ws || { echo "Failed to cd to /workspace/navpilot_ws/src"; exit 1; }
colcon build --packages-select pointcloud_rotation
colcon build --packages-select lidar_imu_sync
colcon build --packages-select robot_description


cd /workspace/navpilot_ws/src/sensor_stack || { echo "Failed to cd to /workspace/navpilot_ws/src/sensor_stack"; exit 1; }
# Clone velodyne repo and checkout humble-devel branch
if [ ! -d "velodyne" ]; then
  git clone https://github.com/ros-drivers/velodyne.git
fi
cd velodyne || exit
git checkout humble-devel
cd ..

# Clone vectornav repo on ros2 branch
if [ ! -d "vectornav" ]; then
  git clone https://github.com/dawonn/vectornav.git -b ros2
fi

# mapping modules
cd /workspace/navpilot_ws/src/mapping_modules || { echo "Failed to cd to /workspace/navpilot_ws/src/mapping_modules"; exit 1; }
if [ ! -d "LIO-SAM" ]; then
    git clone https://github.com/TixiaoShan/LIO-SAM.git
fi
cd LIO-SAM || exit
git checkout ros2
cd ..

# lozalization modules
cd /workspace/navpilot_ws/src/localization_modules || { echo "Failed to cd to /workspace/navpilot_ws/src/localization_modules"; exit 1; }
if [ ! -d "ndt_omp_ros2" ]; then
    git clone https://github.com/rsasaki0109/ndt_omp_ros2
fi

# if [ ! -d "lidar_localization_ros2" ]; then
#     git clone https://github.com/rsasaki0109/lidar_localization_ros2.git
# fi

# hd map modules
cd /workspace/navpilot_ws/src/hdmap_stack || { echo "Failed to cd to /workspace/navpilot_ws/src/hdmap_stack"; exit 1; }
if [ ! -d "mrt_cmake_modules" ]; then
    git clone https://github.com/KIT-MRT/mrt_cmake_modules.git
fi

# Go back to workspace root
cd /workspace/navpilot_ws || { echo "Failed to cd to /workspace/navpilot_ws"; exit 1; }

if [ ! -d "tinyspline" ]; then
    git clone https://github.com/msteinbeck/tinyspline.git
fi

cd tinyspline || { echo "Failed to cd to tinyspline"; exit 1; }

# Build locally (no sudo)
if [ ! -d "build" ]; then
    mkdir build
fi

cd build
cmake -DCMAKE_POSITION_INDEPENDENT_CODE=ON ..
make -j$(nproc)

# Build the specified packages with colcon
colcon build --packages-select vectornav_msgs
colcon build --packages-select vectornav
colcon build --packages-select velodyne_msgs
colcon build --packages-select velodyne_driver
colcon build --packages-select velodyne_laserscan
colcon build --packages-select velodyne_pointcloud
colcon build --packages-select velodyne
colcon build --packages-select lio_sam
colcon build --packages-select ndt_omp_ros2
colcon build --packages-select lidar_localization_ros2
colcon build --packages-select sensors_launch
colcon build --packages-select waypoints_routing
colcon build --packages-select mrt_cmake_modules
colcon build --packages-select polygon_msgs
colcon build --packages-select polygon_rviz_plugins
colcon build --packages-select polygon_demos
colcon build --packages-select polygon_utils
colcon build --packages-select traffic_information_msgs
colcon build --packages-select obstacles_information_msgs
source install/setup.zsh
colcon build --packages-select lanelet2_core
colcon build --packages-select lanelet2_maps
colcon build --packages-select lanelet2_io
colcon build --packages-select lanelet2_projection
colcon build --packages-select lanelet2_traffic_rules
colcon build --packages-select lanelet2_routing
colcon build --packages-select lanelet2_validation
colcon build --packages-select lanelet2_python
colcon build --packages-select lanelet2_examples
source install/setup.zsh
colcon build --packages-select map_visualizer
colcon build --packages-select waypoints_routing
colcon build --packages-select path_planning_dynamic
colcon build --packages-select pointcloud_clustering
colcon build --packages-select pointcloud_clustering_KDTree
colcon build --packages-select pointcloud_rotation
colcon build --packages-select yaw_test
colcon build --packages-select local_path_planning
colcon build --packages-select camera_lidar_sync