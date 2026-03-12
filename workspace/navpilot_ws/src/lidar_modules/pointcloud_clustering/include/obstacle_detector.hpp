#pragma once

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include <pcl/common/io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <utility>
#include <vector>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace lidar_obstacle_detector
{
    struct GridKey
    {
        int x;
        int y;

        bool operator==(const GridKey &other) const
        {
            return x == other.x && y == other.y;
        }
    };

    struct GridKeyHash
    {
        std::size_t operator()(const GridKey &k) const
        {
            // decent integer hash combine
            std::size_t h1 = std::hash<int>{}(k.x);
            std::size_t h2 = std::hash<int>{}(k.y);
            return h1 ^ (h2 + 0x9e3779b9 + (h1 << 6) + (h1 >> 2));
        }
    };

    template <typename PointT>
    class ObstacleDetector
    {
    public:
        using Cloud = pcl::PointCloud<PointT>;
        using CloudPtr = typename Cloud::Ptr;
        using CloudConstPtr = typename Cloud::ConstPtr;

        ObstacleDetector() = default;
        virtual ~ObstacleDetector() = default;

        std::pair<std::vector<CloudPtr>, std::vector<PointT>>
        clustering(const CloudConstPtr &cloud,
                   const float cluster_tolerance,
                   const int min_size,
                   const int max_size);

    private:
        using CellMap = std::unordered_map<GridKey, std::vector<int>, GridKeyHash>;

        static inline GridKey pointToCell(const PointT &p, float cell_size)
        {
            return GridKey{
                static_cast<int>(std::floor(p.x / cell_size)),
                static_cast<int>(std::floor(p.y / cell_size))};
        }
    };

    template <typename PointT>
    std::pair<std::vector<typename ObstacleDetector<PointT>::CloudPtr>, std::vector<PointT>>
    ObstacleDetector<PointT>::clustering(const CloudConstPtr &cloud,
                                         const float cluster_tolerance,
                                         const int min_size,
                                         const int max_size)
    {
        std::vector<CloudPtr> clusters;
        std::vector<PointT> centroids;

        if (!cloud || cloud->empty() || cluster_tolerance <= 0.0f)
            return {clusters, centroids};

        const float cell_size = cluster_tolerance;

        // ------------------------------------------------------------
        // 1) Build grid: cell -> point indices
        // Parallel version uses thread-local maps, then merges.
        // ------------------------------------------------------------
        CellMap grid;
        grid.reserve(cloud->points.size() / 4 + 1);

#ifdef _OPENMP
        int n_threads = omp_get_max_threads();
        std::vector<CellMap> local_maps(n_threads);

#pragma omp parallel
        {
            int tid = omp_get_thread_num();
            auto &local_grid = local_maps[tid];
            local_grid.reserve(cloud->points.size() / (4 * n_threads) + 1);

#pragma omp for nowait
            for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i)
            {
                const auto &p = cloud->points[i];
                if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                    continue;

                GridKey key = pointToCell(p, cell_size);
                local_grid[key].push_back(i);
            }
        }

        for (auto &local_grid : local_maps)
        {
            for (auto &kv : local_grid)
            {
                auto &dst = grid[kv.first];
                dst.insert(dst.end(), kv.second.begin(), kv.second.end());
            }
        }
#else
        for (int i = 0; i < static_cast<int>(cloud->points.size()); ++i)
        {
            const auto &p = cloud->points[i];
            if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z))
                continue;

            GridKey key = pointToCell(p, cell_size);
            grid[key].push_back(i);
        }
#endif

        if (grid.empty())
            return {clusters, centroids};

        // ------------------------------------------------------------
        // 2) Connected components on occupied cells (8-neighborhood)
        // ------------------------------------------------------------
        std::unordered_map<GridKey, bool, GridKeyHash> visited;
        visited.reserve(grid.size());

        std::vector<int> cluster_point_indices;
        cluster_point_indices.reserve(2048);

        for (const auto &entry : grid)
        {
            const GridKey &start_key = entry.first;
            if (visited[start_key])
                continue;

            std::queue<GridKey> q;
            q.push(start_key);
            visited[start_key] = true;

            cluster_point_indices.clear();

            while (!q.empty())
            {
                GridKey current = q.front();
                q.pop();

                auto it = grid.find(current);
                if (it == grid.end())
                    continue;

                // accumulate all point indices in this cell
                const auto &pts = it->second;
                cluster_point_indices.insert(cluster_point_indices.end(), pts.begin(), pts.end());

                // 8-connected neighborhood
                for (int dx = -1; dx <= 1; ++dx)
                {
                    for (int dy = -1; dy <= 1; ++dy)
                    {
                        if (dx == 0 && dy == 0)
                            continue;

                        GridKey neighbor{current.x + dx, current.y + dy};

                        if (visited[neighbor])
                            continue;

                        if (grid.find(neighbor) != grid.end())
                        {
                            bool close = false;
                            const auto &pts_a = it->second;
                            const auto &pts_b = grid[neighbor];

                            for (int ia : pts_a)
                            {
                                const auto &pa = cloud->points[ia];
                                for (int ib : pts_b)
                                {
                                    const auto &pb = cloud->points[ib];
                                    float diff_x = pa.x - pb.x;
                                    float diff_y = pa.y - pb.y;
                                    if (diff_x * diff_x + diff_y * diff_y < cluster_tolerance * cluster_tolerance)
                                    {
                                        close = true;
                                        break;
                                    }
                                }
                                if (close)
                                    break;
                            }

                            if (close)
                            {
                                visited[neighbor] = true;
                                q.push(neighbor);
                            }
                        }
                    }
                }
            }

            const int cluster_size = static_cast<int>(cluster_point_indices.size());
            if (cluster_size < min_size || cluster_size > max_size)
                continue;

            CloudPtr cluster(new Cloud);
            cluster->points.reserve(cluster_size);

            Eigen::Vector4f centroid4(0.f, 0.f, 0.f, 0.f);

            for (int idx : cluster_point_indices)
            {
                const auto &p = cloud->points[idx];
                cluster->points.push_back(p);
                centroid4[0] += p.x;
                centroid4[1] += p.y;
                centroid4[2] += p.z;
            }

            cluster->width = static_cast<std::uint32_t>(cluster->points.size());
            cluster->height = 1;
            cluster->is_dense = true;

            // Filter out oversized clusters (e.g. building walls)
            Eigen::Vector4f min_pt, max_pt;
            pcl::getMinMax3D(*cluster, min_pt, max_pt);
            float width = max_pt.x() - min_pt.x();
            float length = max_pt.y() - min_pt.y();
            if (width > 20.0f || length > 20.0f)
                continue;

            const float inv_n = 1.0f / static_cast<float>(cluster->points.size());
            centroid4 *= inv_n;

            PointT centroid_point;
            centroid_point.x = centroid4[0];
            centroid_point.y = centroid4[1];
            centroid_point.z = centroid4[2];

            clusters.push_back(cluster);
            centroids.push_back(centroid_point);
        }

        return {clusters, centroids};
    }

} // namespace lidar_obstacle_detector