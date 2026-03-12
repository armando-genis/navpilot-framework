#pragma once

#include <pcl/common/common.h>
#include <pcl/common/pca.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/search/kdtree.h>

#include <algorithm>
#include <ctime>
#include <iostream>
#include <string>
#include <unordered_set>
#include <utility>
#include <vector>

namespace lidar_obstacle_detector
{
    template <typename PointT>
    class ObstacleDetector
    {
    public:
        ObstacleDetector();
        virtual ~ObstacleDetector();

        // Clustering function
        std::pair<std::vector<typename pcl::PointCloud<PointT>::Ptr>, std::vector<PointT>>
        clustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const float cluster_tolerance, const int min_size, const int max_size);

    private:
        typename pcl::search::KdTree<PointT>::Ptr tree_;
    };

    // constructor:
    template <typename PointT>
    ObstacleDetector<PointT>::ObstacleDetector() : tree_(new pcl::search::KdTree<PointT>()) {}

    // de-constructor:
    template <typename PointT>
    ObstacleDetector<PointT>::~ObstacleDetector() {}

    template <typename PointT>
    std::pair<std::vector<typename pcl::PointCloud<PointT>::Ptr>, std::vector<PointT>>
    ObstacleDetector<PointT>::clustering(const typename pcl::PointCloud<PointT>::ConstPtr &cloud, const float cluster_tolerance, const int min_size, const int max_size)
    {
        // Time clustering process
        // const auto start_time = std::chrono::steady_clock::now();

        std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters;

        std::vector<PointT> centroids;

        // Perform euclidean clustering to group detected obstacles (reuse persistent tree)
        tree_->setInputCloud(cloud);

        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<PointT> ec;
        ec.setClusterTolerance(cluster_tolerance);
        ec.setMinClusterSize(min_size);
        ec.setMaxClusterSize(max_size);
        ec.setSearchMethod(tree_);
        ec.setInputCloud(cloud);
        ec.extract(cluster_indices);

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(cloud);

        for (const auto &indices : cluster_indices)
        {
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices(indices));
            typename pcl::PointCloud<PointT>::Ptr cluster(new pcl::PointCloud<PointT>);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cluster);
            clusters.push_back(cluster);

            // Eigen::Vector4f centroid;
            // pcl::compute3DCentroid(*cluster, centroid);

            // PointT centroid_point;
            // centroid_point.x = centroid[0];
            // centroid_point.y = centroid[1];
            // centroid_point.z = centroid[2];
            // centroids.push_back(centroid_point);
        }

        return {clusters, centroids};
    }

}