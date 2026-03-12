#ifndef POINTCLOUD_CLUSTERING_NODE_H
#define POINTCLOUD_CLUSTERING_NODE_H

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/polygon.hpp>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/surface/concave_hull.h>

#include "obstacles_information_msgs/msg/obstacle.hpp"
#include "obstacles_information_msgs/msg/obstacle_collection.hpp"

#include "obstacle_detector.hpp"

#include <memory>
#include <vector>
#include <string>

class pointcloud_clustering_node : public rclcpp::Node
{
private:
    // parameters
    double GROUND_THRESHOLD{};
    double CLUSTER_THRESH{};
    int CLUSTER_MAX_SIZE{};
    int CLUSTER_MIN_SIZE{};
    bool USE_PCA_BOX{};
    double DISPLACEMENT_THRESH{};
    double IOU_THRESH{};
    bool USE_TRACKING{};
    double HULL_ALPHA{};
    bool USE_CONCAVE_HULL{};

    std::shared_ptr<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>> obstacle_detector_;
    obstacles_information_msgs::msg::ObstacleCollection obstacle_collection_;

    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud_;

    void pointCloudCallback(const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg);
    void buildAndPublishHulls(const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters,
                              const std_msgs::msg::Header &header);
    void imaginaryObstacle(const std_msgs::msg::Header &header);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr sub_points_cloud_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr hull_publisher_;
    rclcpp::Publisher<obstacles_information_msgs::msg::ObstacleCollection>::SharedPtr obstacle_info_publisher_;

public:
    pointcloud_clustering_node();
    ~pointcloud_clustering_node();
};

#endif