#include "LidarCameraTree.hpp"

namespace multicamera_detection
{

void LidarCameraTree::build(
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const std::vector<std::shared_ptr<CameraLidarExtrinsics>>& extrinsics,
    const std::vector<std::shared_ptr<CameraUndistorter>>& intrinsics,
    int img_w, int img_h)
{
  cameras_.clear();
  cameras_.resize(extrinsics.size());

  for (size_t ci = 0; ci < extrinsics.size(); ci++)
  {
    auto& cam_tree = cameras_[ci];
    cam_tree.valid = false;

    if (!extrinsics[ci] || ci >= intrinsics.size() || !intrinsics[ci]) continue;
    if (!cloud || cloud->empty()) continue;

    cam_tree.R = extrinsics[ci]->get_R_opencv();
    cam_tree.t = extrinsics[ci]->get_t_opencv();
    cam_tree.K = intrinsics[ci]->get_K();

    const double fx = cam_tree.K.at<double>(0, 0);
    const double fy = cam_tree.K.at<double>(1, 1);
    const double cx = cam_tree.K.at<double>(0, 2);
    const double cy = cam_tree.K.at<double>(1, 2);

    cam_tree.pixels = std::make_shared<pcl::PointCloud<pcl::PointXY>>();
    cam_tree.xyz.clear();

    for (const auto& p : cloud->points)
    {
      // Transform to camera frame
      cv::Mat pt  = (cv::Mat_<double>(3,1) << p.x, p.y, p.z);
      cv::Mat cam = cam_tree.R * pt + cam_tree.t;

      double X = cam.at<double>(0);
      double Y = cam.at<double>(1);
      double Z = cam.at<double>(2);

      if (Z <= 0.1) continue;   // behind camera

      float u = static_cast<float>(fx * X / Z + cx);
      float v = static_cast<float>(fy * Y / Z + cy);

      // Keep only points that project inside the image (with small margin)
      if (u < -20 || u >= img_w + 20 || v < -20 || v >= img_h + 20) continue;

      pcl::PointXY px;
      px.x = u; px.y = v;
      cam_tree.pixels->push_back(px);
      cam_tree.xyz.push_back({p.x, p.y, p.z});
    }

    if (cam_tree.pixels->empty()) continue;

    cam_tree.tree.setInputCloud(cam_tree.pixels);
    cam_tree.valid = true;
  }
}

std::optional<cv::Point3f> LidarCameraTree::query(
    int cam_idx, float u, float v,
    float max_ray_dist, int k_candidates) const
{
  if (cam_idx < 0 || static_cast<size_t>(cam_idx) >= cameras_.size()) return std::nullopt;
  const auto& c = cameras_[cam_idx];
  if (!c.valid || c.pixels->empty()) return std::nullopt;

  // KDTree search in pixel space
  pcl::PointXY query_pt;
  query_pt.x = u; query_pt.y = v;

  std::vector<int>   indices(k_candidates);
  std::vector<float> sq_dists(k_candidates);
  int found = c.tree.nearestKSearch(query_pt, k_candidates, indices, sq_dists);
  if (found == 0) return std::nullopt;

  // Build ray from camera center through pixel (u, v)
  const double fx = c.K.at<double>(0, 0);
  const double fy = c.K.at<double>(1, 1);
  const double cx = c.K.at<double>(0, 2);
  const double cy = c.K.at<double>(1, 2);

  cv::Mat ray_cam = (cv::Mat_<double>(3,1)
      << (u - cx) / fx,
         (v - cy) / fy,
         1.0);
  double norm = cv::norm(ray_cam);
  ray_cam /= norm;

  // Ray and origin in LiDAR frame:  R^T * ray_cam,  origin = -R^T * t
  cv::Mat Rt         = c.R.t();
  cv::Mat ray_lidar  = Rt * ray_cam;
  cv::Mat origin_mat = -Rt * c.t;

  cv::Point3f origin(
      static_cast<float>(origin_mat.at<double>(0)),
      static_cast<float>(origin_mat.at<double>(1)),
      static_cast<float>(origin_mat.at<double>(2)));

  cv::Point3f ray_dir(
      static_cast<float>(ray_lidar.at<double>(0)),
      static_cast<float>(ray_lidar.at<double>(1)),
      static_cast<float>(ray_lidar.at<double>(2)));

  // Filter candidates by perpendicular distance to ray
  float   best_depth = std::numeric_limits<float>::max();
  cv::Point3f best_pt;
  bool    found_any = false;

  for (int i = 0; i < found; i++)
  {
    const cv::Point3f& p = c.xyz[indices[i]];
    cv::Point3f vec = p - origin;

    float depth = vec.dot(ray_dir);       // projection along ray
    if (depth <= 0.0f) continue;          // behind camera origin

    cv::Point3f closest = origin + ray_dir * depth;
    float dist_ray = static_cast<float>(cv::norm(p - closest));

    if (dist_ray < max_ray_dist && depth < best_depth)
    {
      best_depth = depth;
      best_pt    = p;
      found_any  = true;
    }
  }

  if (!found_any) return std::nullopt;
  return best_pt;
}

}  // namespace multicamera_detection