#pragma once
#include <vector>
#include <optional>
#include <opencv2/opencv.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>

#include "CameraLidarExtrinsics.hpp"
#include "CameraUndistorter.hpp"

namespace multicamera_detection
{

struct PointEntry {
  float u, v;         // projected pixel
  float x, y, z;     // original LiDAR world coords
};

class LidarCameraTree
{
public:
  // Call once per new cloud — builds one KDTree per camera
  void build(
      const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
      const std::vector<std::shared_ptr<CameraLidarExtrinsics>>& extrinsics,
      const std::vector<std::shared_ptr<CameraUndistorter>>& intrinsics,
      int img_w, int img_h);

  // Query: given a pixel (u,v) in camera cam_idx, return the nearest 3D LiDAR point
  // Returns nullopt if no point found within max_ray_dist
  std::optional<cv::Point3f> query(
      int cam_idx, float u, float v,
      float max_ray_dist = 0.3f, int k_candidates = 20) const;

private:
  struct PerCamera {
    pcl::KdTreeFLANN<pcl::PointXY>           tree;
    pcl::PointCloud<pcl::PointXY>::Ptr        pixels;   // 2D projected pixels
    std::vector<cv::Point3f>                  xyz;      // matching 3D coords
    cv::Mat                                   R, t, K;  // stored for ray cast
    bool                                      valid = false;
  };

  std::vector<PerCamera> cameras_;
};

}  // namespace multicamera_detection