#include "pointcloud_clustering_node.h"

pointcloud_clustering_node::pointcloud_clustering_node()
    : Node("pointcloud_clustering_node")
{
    this->declare_parameter("GROUND_THRESHOLD", 0.0);
    this->declare_parameter("CLUSTER_THRESH", 0.5);
    this->declare_parameter("CLUSTER_MAX_SIZE", 100000);
    this->declare_parameter("CLUSTER_MIN_SIZE", 8);
    this->declare_parameter("USE_PCA_BOX", false);
    this->declare_parameter("DISPLACEMENT_THRESH", 0.0);
    this->declare_parameter("IOU_THRESH", 0.0);
    this->declare_parameter("USE_TRACKING", false);
    this->declare_parameter("HULL_ALPHA", 1.0);
    this->declare_parameter("USE_CONCAVE_HULL", false);

    this->get_parameter("GROUND_THRESHOLD", GROUND_THRESHOLD);
    this->get_parameter("CLUSTER_THRESH", CLUSTER_THRESH);
    this->get_parameter("CLUSTER_MAX_SIZE", CLUSTER_MAX_SIZE);
    this->get_parameter("CLUSTER_MIN_SIZE", CLUSTER_MIN_SIZE);
    this->get_parameter("USE_PCA_BOX", USE_PCA_BOX);
    this->get_parameter("DISPLACEMENT_THRESH", DISPLACEMENT_THRESH);
    this->get_parameter("IOU_THRESH", IOU_THRESH);
    this->get_parameter("USE_TRACKING", USE_TRACKING);
    this->get_parameter("HULL_ALPHA", HULL_ALPHA);
    this->get_parameter("USE_CONCAVE_HULL", USE_CONCAVE_HULL);

    sub_points_cloud_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        "/velodyne_points_rotated_notground",
        rclcpp::SensorDataQoS(),
        std::bind(&pointcloud_clustering_node::pointCloudCallback, this, std::placeholders::_1));

    hull_publisher_ =
        this->create_publisher<visualization_msgs::msg::MarkerArray>("/hull_marker", 10);

    obstacle_info_publisher_ =
        this->create_publisher<obstacles_information_msgs::msg::ObstacleCollection>("/obstacle_info", 10);

    obstacle_detector_ =
        std::make_shared<lidar_obstacle_detector::ObstacleDetector<pcl::PointXYZ>>();

    input_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());

    obstacle_collection_.obstacles.reserve(128);

    RCLCPP_INFO(this->get_logger(), "----> pointcloud_clustering_node initialized");
    RCLCPP_INFO(this->get_logger(), "----> GROUND_THRESHOLD: %.3f", GROUND_THRESHOLD);
    RCLCPP_INFO(this->get_logger(), "----> CLUSTER_THRESH: %.3f", CLUSTER_THRESH);
    RCLCPP_INFO(this->get_logger(), "----> CLUSTER_MAX_SIZE: %d", CLUSTER_MAX_SIZE);
    RCLCPP_INFO(this->get_logger(), "----> CLUSTER_MIN_SIZE: %d", CLUSTER_MIN_SIZE);
    RCLCPP_INFO(this->get_logger(), "----> DISPLACEMENT_THRESH: %.3f", DISPLACEMENT_THRESH);
    RCLCPP_INFO(this->get_logger(), "----> IOU_THRESH: %.3f", IOU_THRESH);
    RCLCPP_INFO(this->get_logger(), "----> USE_TRACKING: %d", USE_TRACKING);
    RCLCPP_INFO(this->get_logger(), "----> USE_CONCAVE_HULL: %d", USE_CONCAVE_HULL);
    RCLCPP_INFO(this->get_logger(), "----> HULL_ALPHA: %.3f", HULL_ALPHA);
}

pointcloud_clustering_node::~pointcloud_clustering_node() {}

void pointcloud_clustering_node::pointCloudCallback(
    const sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
    input_cloud_->clear();
    pcl::fromROSMsg(*msg, *input_cloud_);

    if (input_cloud_->empty())
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "Received empty point cloud");
        imaginaryObstacle(msg->header);
        return;
    }

    try
    {
        auto cloud_clusters = obstacle_detector_->clustering(
            input_cloud_, static_cast<float>(CLUSTER_THRESH),
            CLUSTER_MIN_SIZE, CLUSTER_MAX_SIZE);

        auto &clusters = cloud_clusters.first;

        if (!clusters.empty())
        {
            buildAndPublishHulls(clusters, msg->header);
        }
        else
        {
            imaginaryObstacle(msg->header);
        }
    }
    catch (const std::exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error processing point cloud: %s", e.what());
        imaginaryObstacle(msg->header);
    }
}

void pointcloud_clustering_node::buildAndPublishHulls(
    const std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> &cloud_clusters,
    const std_msgs::msg::Header &header)
{
    visualization_msgs::msg::MarkerArray hull_markers;
    hull_markers.markers.reserve(cloud_clusters.size());

    obstacle_collection_.obstacles.clear();
    obstacle_collection_.header = header;

    const rclcpp::Time stamp = this->now();

#ifdef _OPENMP
#pragma omp parallel
    {
        std::vector<visualization_msgs::msg::Marker> local_markers;
        std::vector<obstacles_information_msgs::msg::Obstacle> local_obstacles;

#pragma omp for nowait
        for (int i = 0; i < static_cast<int>(cloud_clusters.size()); ++i)
        {
            const auto &cluster = cloud_clusters[i];
            if (!cluster || cluster->empty())
                continue;

            pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>());

            if (USE_CONCAVE_HULL)
            {
                pcl::ConcaveHull<pcl::PointXYZ> hull;
                hull.setInputCloud(cluster);
                hull.setDimension(2);
                hull.setAlpha(HULL_ALPHA);
                hull.reconstruct(*hull_cloud);
            }
            else
            {
                pcl::ConvexHull<pcl::PointXYZ> hull;
                hull.setInputCloud(cluster);
                hull.setDimension(2);
                hull.reconstruct(*hull_cloud);
            }

            if (!hull_cloud || hull_cloud->points.size() < 3)
                continue;

            obstacles_information_msgs::msg::Obstacle obstacle;
            geometry_msgs::msg::Polygon polygon;
            polygon.points.reserve(hull_cloud->points.size() + 1);

            visualization_msgs::msg::Marker marker;
            marker.header = header;
            marker.header.stamp = stamp;
            marker.ns = "hull";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.scale.x = 0.07;
            marker.color.r = 1.0;
            marker.color.g = 1.0;
            marker.color.b = 1.0;
            marker.color.a = 1.0;
            marker.points.reserve(hull_cloud->points.size() + 1);

            for (const auto &point : hull_cloud->points)
            {
                geometry_msgs::msg::Point32 p32;
                p32.x = point.x;
                p32.y = point.y;
                p32.z = 0.0f;
                polygon.points.push_back(p32);

                geometry_msgs::msg::Point p;
                p.x = point.x;
                p.y = point.y;
                p.z = 0.0;
                marker.points.push_back(p);
            }

            // close polygon and marker strip
            polygon.points.push_back(polygon.points.front());
            marker.points.push_back(marker.points.front());

            obstacle.polygon = polygon;
            obstacle.id = i;
            obstacle.type = "NONE";

            local_obstacles.push_back(std::move(obstacle));
            local_markers.push_back(std::move(marker));
        }

#pragma omp critical
        {
            obstacle_collection_.obstacles.insert(
                obstacle_collection_.obstacles.end(),
                local_obstacles.begin(), local_obstacles.end());

            hull_markers.markers.insert(
                hull_markers.markers.end(),
                local_markers.begin(), local_markers.end());
        }
    }
#else
    for (int i = 0; i < static_cast<int>(cloud_clusters.size()); ++i)
    {
        const auto &cluster = cloud_clusters[i];
        if (!cluster || cluster->empty())
            continue;

        pcl::PointCloud<pcl::PointXYZ>::Ptr hull_cloud(new pcl::PointCloud<pcl::PointXYZ>());

        if (USE_CONCAVE_HULL)
        {
            pcl::ConcaveHull<pcl::PointXYZ> hull;
            hull.setInputCloud(cluster);
            hull.setDimension(2);
            hull.setAlpha(HULL_ALPHA);
            hull.reconstruct(*hull_cloud);
        }
        else
        {
            pcl::ConvexHull<pcl::PointXYZ> hull;
            hull.setInputCloud(cluster);
            hull.setDimension(2);
            hull.reconstruct(*hull_cloud);
        }

        if (!hull_cloud || hull_cloud->points.size() < 3)
            continue;

        obstacles_information_msgs::msg::Obstacle obstacle;
        geometry_msgs::msg::Polygon polygon;
        polygon.points.reserve(hull_cloud->points.size() + 1);

        visualization_msgs::msg::Marker marker;
        marker.header = header;
        marker.header.stamp = stamp;
        marker.ns = "hull";
        marker.id = i;
        marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.07;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.color.a = 1.0;
        marker.points.reserve(hull_cloud->points.size() + 1);

        for (const auto &point : hull_cloud->points)
        {
            geometry_msgs::msg::Point32 p32;
            p32.x = point.x;
            p32.y = point.y;
            p32.z = 0.0f;
            polygon.points.push_back(p32);

            geometry_msgs::msg::Point p;
            p.x = point.x;
            p.y = point.y;
            p.z = 0.0;
            marker.points.push_back(p);
        }

        polygon.points.push_back(polygon.points.front());
        marker.points.push_back(marker.points.front());

        obstacle.polygon = polygon;
        obstacle.id = i;
        obstacle.type = "NONE";

        obstacle_collection_.obstacles.push_back(std::move(obstacle));
        hull_markers.markers.push_back(std::move(marker));
    }
#endif

    if (!obstacle_collection_.obstacles.empty())
    {
        hull_publisher_->publish(hull_markers);
        obstacle_info_publisher_->publish(obstacle_collection_);
    }
    else
    {
        imaginaryObstacle(header);
    }
}

void pointcloud_clustering_node::imaginaryObstacle(const std_msgs::msg::Header &header)
{
    obstacle_collection_.obstacles.clear();
    obstacle_collection_.header = header;

    obstacles_information_msgs::msg::Obstacle obstacle;
    geometry_msgs::msg::Polygon polygon;
    geometry_msgs::msg::Point32 p;

    p.x = 5000.0f;
    p.y = 0.0f;
    p.z = 0.0f;

    polygon.points.push_back(p);
    obstacle.polygon = polygon;
    obstacle.id = 1;
    obstacle.type = "NONE";

    obstacle_collection_.obstacles.push_back(obstacle);
    obstacle_info_publisher_->publish(obstacle_collection_);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<pointcloud_clustering_node>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}