#ifndef POINTCLOUD_CLUSTERING_NODE_H
#define POINTCLOUD_CLUSTERING_NODE_H
// RORS2
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>

// PCL
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/convex_hull.h>

// Custom msgs obstacles_information_msgs for Obstacle and ObstacleCollection
#include "obstacles_information_msgs/msg/obstacle.hpp"
#include "obstacles_information_msgs/msg/obstacle_collection.hpp"

#include "obstacle_detector.hpp"

#include <vector>
#include <chrono>
#include <iostream>

using namespace std;

class pointcloud_clustering_node : public rclcpp::Node
{
private:
    // colors for the terminal
    std::string green = "\033[1;32m";
    std::string red = "\033[1;31m";
    std::string blue = "\033[1;34m";
    std::string yellow = "\033[1;33m";
    std::string purple = "\033[1;35m";
    std::string reset = "\033[0m";

    // variables
    double GROUND_THRESHOLD;
    double CLUSTER_THRESH;
    int CLUSTER_MAX_SIZE;
    int CLUSTER_MIN_SIZE;
    bool USE_PCA_BOX;
    double DISPLACEMENT_THRESH;
    double IOU_THRESH;
    bool USE_TRACKING;

    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector;
    obstacles_information_msgs::msg::ObstacleCollection obstacle_collection;

    // Point Cloud callback
    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void convex_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters);

    // subscriber & publisher
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hull_publisher_;
    rclcpp::Publisher<obstacles_information_msgs::msg::ObstacleCollection>::SharedPtr obstacle_info_publisher_;

public:
    pointcloud_clustering_node(/* args */);
    ~pointcloud_clustering_node();
};

#endif // POINTCLOUD_CLUSTERING_NODE_H