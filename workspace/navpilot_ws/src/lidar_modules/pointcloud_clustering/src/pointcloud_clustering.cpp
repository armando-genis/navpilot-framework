#include "pointcloud_clustering_node.h"

pointcloud_clustering_node::pointcloud_clustering_node(/* args */) : Node("pointcloud_clustering_node")
{
    // Parameters
    this->declare_parameter("GROUND_THRESHOLD", 0.0);
    this->declare_parameter("CLUSTER_THRESH", 0.0);
    this->declare_parameter("CLUSTER_MAX_SIZE", 0);
    this->declare_parameter("CLUSTER_MIN_SIZE", 0);
    this->declare_parameter("USE_PCA_BOX", false);
    this->declare_parameter("DISPLACEMENT_THRESH", 0.0);
    this->declare_parameter("IOU_THRESH", 0.0);
    this->declare_parameter("USE_TRACKING", false);

    // Get parameters
    this->get_parameter("GROUND_THRESHOLD", GROUND_THRESHOLD);
    this->get_parameter("CLUSTER_THRESH", CLUSTER_THRESH);
    this->get_parameter("CLUSTER_MAX_SIZE", CLUSTER_MAX_SIZE);
    this->get_parameter("CLUSTER_MIN_SIZE", CLUSTER_MIN_SIZE);
    this->get_parameter("USE_PCA_BOX", USE_PCA_BOX);
    this->get_parameter("DISPLACEMENT_THRESH", DISPLACEMENT_THRESH);
    this->get_parameter("IOU_THRESH", IOU_THRESH);
    this->get_parameter("USE_TRACKING", USE_TRACKING);

    // Create subscriber
    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>("/points_rotated_notground", 10, std::bind(&pointcloud_clustering_node::pointCloudCallback, this, std::placeholders::_1));
    // hull_publisher_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("/hull_marker", 10);
    obstacle_info_publisher_ = this->create_publisher<obstacles_information_msgs::msg::ObstacleCollection>("/obstacle_info", 10);

    // Create point processor
    obstacle_detector = std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();

    RCLCPP_INFO(this->get_logger(), "\033[1;32m----> lidar3d_Clustering_node initialized.\033[0m");
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> GROUND_THRESHOLD: %f\033[0m", GROUND_THRESHOLD);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_THRESH: %f\033[0m", CLUSTER_THRESH);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_MAX_SIZE: %d\033[0m", CLUSTER_MAX_SIZE);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> CLUSTER_MIN_SIZE: %d\033[0m", CLUSTER_MIN_SIZE);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> DISPLACEMENT_THRESH: %f\033[0m", DISPLACEMENT_THRESH);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> IOU_THRESH: %f\033[0m", IOU_THRESH);
    RCLCPP_INFO(this->get_logger(), "\033[1;34m----> USE_TRACKING: %d\033[0m", USE_TRACKING);
}

pointcloud_clustering_node::~pointcloud_clustering_node()
{
}

// Point Cloud callback
void pointcloud_clustering_node::pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{

    // Convert ROS PointCloud2 to PCL PointCloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*msg, *input_cloud);

    // Check if the input cloud is empty
    if (input_cloud->empty())
    {
        std::cout << red << "Received empty point cloud" << reset << std::endl;
        imaginaryObstacle();
        return;
    }

    try
    {
        auto cloud_clusters = obstacle_detector->clustering(input_cloud, CLUSTER_THRESH, CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);
        auto &clusters = cloud_clusters.first;

        if (!clusters.empty())
        {
            // std::cout << green << "Number of clusters: " << clusters.size() << reset << std::endl;
            convex_hull(clusters);
        }
        else
        {
            std::cout << yellow << "No clusters found" << reset << std::endl;
            imaginaryObstacle();
        }
    }
    catch (const std::exception &e)
    {
        std::cout << red << "Error processing point cloud: " << e.what() << reset << std::endl;
    }
}

void pointcloud_clustering_node::convex_hull(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> cloud_clusters)
{
    visualization_msgs::msg::MarkerArray hull_markers;
    obstacle_collection.obstacles.clear();

    obstacle_collection.header.stamp = rclcpp::Clock{}.now();
    obstacle_collection.header.frame_id = "velodyne";

    int index = 0; // Declare an index variable
    for (auto &cluster : cloud_clusters)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr convexHull(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::ConvexHull<pcl::PointXYZ> hull;
        hull.setInputCloud(cluster);
        hull.setDimension(2);
        hull.reconstruct(*convexHull);

        if (convexHull->empty())
        {
            std::cout << red << "Convex hull is empty" << reset << std::endl;
            continue;
        }
        if (hull.getDimension() == 2)
        {
            // std::vector<geometry_msgs::msg::Point> hull_points;
            obstacles_information_msgs::msg::Obstacle obstacle;
            geometry_msgs::msg::Polygon polygon;

            for (const auto &point : convexHull->points)
            {
                geometry_msgs::msg::Point32 p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0.0;
                polygon.points.push_back(p);

                // geometry_msgs::msg::Point hull_point;
                // hull_point.x = p.x;
                // hull_point.y = p.y;
                // hull_point.z = p.z;
                // hull_points.push_back(hull_point);
            }

            // Close the loop
            // hull_points.push_back(hull_points.front());
            polygon.points.push_back(polygon.points.front());

            obstacle.polygon = polygon;
            obstacle.id = index;
            obstacle.type = "NONE";

            obstacle_collection.obstacles.push_back(obstacle);

            // Create a marker for the convex hull
            // visualization_msgs::msg::Marker hull_marker;
            // hull_marker.header.frame_id = "velodyne";
            // hull_marker.header.stamp = this->now();
            // hull_marker.ns = "hull";
            // hull_marker.id = index;
            // hull_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            // hull_marker.action = visualization_msgs::msg::Marker::ADD;
            // hull_marker.scale.x = 0.07;
            // hull_marker.color.r = 1.0;
            // hull_marker.color.g = 1.0;
            // hull_marker.color.b = 1.0;
            // hull_marker.color.a = 1.0;
            // hull_marker.points = hull_points;

            // hull_markers.markers.push_back(hull_marker);

            index++;
        }
    }
    // Publish the convex hull
    if (!obstacle_collection.obstacles.empty())
    {
        // hull_publisher_->publish(hull_markers);
        // std::cout << yellow << "Convex hull markers published" << reset << std::endl;
        // size of the obstacle collection
        // std::cout << yellow << "Obstacle collection size: " << obstacle_collection.obstacles.size() << reset << std::endl;
        obstacle_info_publisher_->publish(obstacle_collection);
    }
    else
    {
        imaginaryObstacle();
    }
}

void pointcloud_clustering_node::imaginaryObstacle()
{
    // This function can be implemented to create imaginary obstacles for convenience
    // > If clustering dont detect any obstacle, this function can create a static obstacle far from the robot
    // > Or is useful if pointcloud is empty
    obstacle_collection.obstacles.clear();
    obstacle_collection.header.stamp = rclcpp::Clock{}.now();
    obstacle_collection.header.frame_id = "base_link";

    obstacles_information_msgs::msg::Obstacle obstacle;
    geometry_msgs::msg::Polygon polygon;
    geometry_msgs::msg::Point32 p;
    p.x = 5000.0;  // <--- 5km in front of the robot
    p.y = 0.0;
    p.z = 0.0;
    polygon.points.push_back(p);
    obstacle.polygon = polygon;
    obstacle.id = 1;
    obstacle.type = "NONE";
    
    // Add the imaginary obstacle to the collection
    obstacle_collection.obstacles.push_back(obstacle);
    std::cout << yellow << "Imaginary Obstacle published " << reset << std::endl;
    obstacle_info_publisher_->publish(obstacle_collection);

}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_clustering_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}