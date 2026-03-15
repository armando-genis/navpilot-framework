#include "multicamera_subscriber.hpp"

#include <iomanip>
#include <fstream>
#include <sstream>

#include <tf2/exceptions.h>
#include <geometry_msgs/msg/transform.hpp>

namespace path_processing
{

namespace
{
void printMatLikePython(std::ostream& out, const cv::Mat& M, const char* name)
{
  out << name << ":\n";
  if (M.rows == 3 && M.cols == 3)
  {
    for (int r = 0; r < 3; r++)
    {
      out << (r == 0 ? "[[" : " [");
      for (int c = 0; c < 3; c++)
        out << std::setw(12) << std::fixed << std::setprecision(8) << M.at<double>(r, c);
      out << (r == 2 ? "]]" : "]\n");
    }
    out << "\n\n";
  }
  else if (M.rows == 3 && M.cols == 1)
  {
    out << "[[ " << std::setw(11) << std::fixed << std::setprecision(8) << M.at<double>(0, 0) << "]\n";
    out << " [" << std::setw(12) << std::fixed << std::setprecision(8) << M.at<double>(1, 0) << "]\n";
    out << " [" << std::setw(12) << std::fixed << std::setprecision(8) << M.at<double>(2, 0) << "]]\n\n";
  }
}

void printMatScientific(std::ostream& out, const cv::Mat& M)
{
  out << std::scientific << std::setprecision(8);
  if (M.rows == 3 && M.cols == 3)
  {
    for (int r = 0; r < 3; r++)
    {
      out << (r == 0 ? "[[" : " [");
      for (int c = 0; c < 3; c++)
        out << std::setw(17) << M.at<double>(r, c);
      out << (r == 2 ? "]]" : "]\n");
    }
    out << "\n\n";
  }
  else if (M.rows == 4 && M.cols == 1)
  {
    for (int r = 0; r < 4; r++)
    {
      out << (r == 0 ? "[[ " : " [") << std::setw(12) << M.at<double>(r, 0);
      out << (r == 3 ? "]]" : " ]") << "\n";
    }
    out << "\n";
  }
  out << std::fixed;
}
}  // namespace

MultiCameraSubscriber::MultiCameraSubscriber()
  : Node("path_processing_node"),
    tf2_buffer_(this->get_clock()),
    tf2_listener_(tf2_buffer_)
{
  this->declare_parameter<int>("num_cameras", 3);
  this->declare_parameter<std::string>("calib_dir", "");
  this->declare_parameter<std::string>("waypoints_file_path", "");
  this->declare_parameter<std::string>("global_frame_id", "map");
  this->declare_parameter<std::string>("robot_frame_id", "velodyne");

  num_cameras_ = this->get_parameter("num_cameras").as_int();
  calib_dir_ = this->get_parameter("calib_dir").as_string();
  waypoints_file_path_ = this->get_parameter("waypoints_file_path").as_string();
  global_frame_id_ = this->get_parameter("global_frame_id").as_string();
  robot_frame_id_ = this->get_parameter("robot_frame_id").as_string();

  calib.load(calib_dir_);
  readWaypoints();

  if (static_cast<size_t>(0) < calib.camera_array.size() && calib.camera_array[0])
  {
    auto* cam = calib.camera_array[0].get();
    std::cout << "\nIntrinsics: LOADED\n\n";
    cv::Size sz = cam->get_frame_size();
    std::cout << "Frame size (w x h): (" << sz.width << ", " << sz.height << ")\n\n";
    std::cout << "Intrinsics (K):\n";
    printMatScientific(std::cout, cam->get_K());
    std::cout << "Distortion (D):\n";
    printMatScientific(std::cout, cam->get_D());
  }

  if (static_cast<size_t>(0) < calib.extrinsics_array.size() && calib.extrinsics_array[0])
  {
    auto* ext = calib.extrinsics_array[0].get();
    std::cout << "Extrinsics (Lidar \u2192 Camera): LOADED\n\n";
    printMatLikePython(std::cout, ext->get_R_opencv(), "R (opencv_frame)");
    printMatLikePython(std::cout, ext->get_t_opencv(), "t (opencv_frame)");
    printMatLikePython(std::cout, ext->get_R_robot(), "R (robot_frame)");
    printMatLikePython(std::cout, ext->get_t_robot(), "t (robot_frame)");
    const auto& lr = ext->get_lidar_rotation();
    std::cout << "Lidar rotation: {'axis_x': " << std::fixed << std::setprecision(8) << lr.axis_x
              << ", 'axis_y': " << lr.axis_y << ", 'axis_z': " << lr.axis_z << "}\n" << std::endl;
  }

  auto qos = rclcpp::SensorDataQoS();

  // Subscribe to camera topics
  for (int i = 0; i < num_cameras_; i++)
  {
    std::string topic = "/racecar/camera/camera_" + std::to_string(i) + "/image_raw";
    const int camera_id = i;

    auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        topic,
        qos,
        [this, camera_id](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          imageCallback(msg, camera_id);
        }
    );

    subscribers_.push_back(sub);

    RCLCPP_INFO(this->get_logger(), "Subscribed to %s", topic.c_str());
  }

  cv::namedWindow("MultiCamera", cv::WINDOW_NORMAL);

  frame_cache_.resize(num_cameras_);
  frame_ready_.assign(num_cameras_, false);
}

void MultiCameraSubscriber::readWaypoints()
{
  if (waypoints_file_path_.empty())
  {
    RCLCPP_WARN(this->get_logger(), "Waypoints file path is empty, skipping load");
    return;
  }

  std::ifstream file(waypoints_file_path_);
  if (!file.is_open())
  {
    RCLCPP_ERROR(this->get_logger(), "Failed to open file: %s", waypoints_file_path_.c_str());
    return;
  }

  std::string line;
  // Skip the header line
  std::getline(file, line);

  waypoints_.clear();
  while (std::getline(file, line))
  {
    std::stringstream ss(line);
    std::string value;
    Eigen::VectorXd waypoint(6);  // x, y, z, yaw, velocity, change_flag

    for (int i = 0; i < 6; ++i)
    {
      std::getline(ss, value, ',');
      waypoint(i) = std::stod(value);
    }

    waypoints_.push_back(waypoint);
  }

  file.close();
  RCLCPP_INFO(this->get_logger(), "Loaded %zu waypoints from %s", waypoints_.size(), waypoints_file_path_.c_str());
}

bool MultiCameraSubscriber::getCurrentRobotPose()
{
  geometry_msgs::msg::Transform pose_tf;
  try
  {
    pose_tf = tf2_buffer_.lookupTransform(global_frame_id_, robot_frame_id_, tf2::TimePointZero).transform;
    current_pose_.header.frame_id = global_frame_id_;
    current_pose_.header.stamp = this->now();
    current_pose_.pose.pose.position.x = pose_tf.translation.x;
    current_pose_.pose.pose.position.y = pose_tf.translation.y;
    current_pose_.pose.pose.position.z = pose_tf.translation.z;
    current_pose_.pose.pose.orientation = pose_tf.rotation;
    return true;
  }
  catch (const tf2::TransformException& ex)
  {
    RCLCPP_WARN_THROTTLE(
      this->get_logger(), *this->get_clock(), 1000,
      "Transform unavailable (%s -> %s): %s. Skipping until TF is available.",
      global_frame_id_.c_str(), robot_frame_id_.c_str(), ex.what());
    return false;
  }
}

void MultiCameraSubscriber::imageCallback(
    sensor_msgs::msg::Image::ConstSharedPtr msg, int camera_id)
{
  try
  {
    cv::Mat frame;

    if (msg->encoding == "mjpeg")
    {
      std::vector<uint8_t> buffer(msg->data.begin(), msg->data.end());
      frame = cv::imdecode(buffer, cv::IMREAD_COLOR);
    }
    else
    {
      RCLCPP_WARN(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
      return;
    }

    if (frame.empty())
      return;

    // Undistort image
    if (static_cast<size_t>(camera_id) < calib.camera_array.size())
    {
      auto cam = calib.camera_array[camera_id];

      if (cam)
      {
        cam->ensure_size(frame.cols, frame.rows);
        frame = cam->undistort(frame);
      }
    }

    cv::putText(
        frame,
        "Camera " + std::to_string(camera_id),
        {20, 40},
        cv::FONT_HERSHEY_SIMPLEX,
        1.0,
        {0, 255, 0},
        2
    );

    // At the end of imageCallback, replace imshow with:
    frame_cache_[camera_id] = frame;
    frame_ready_[camera_id] = true;

    if (std::all_of(frame_ready_.begin(), frame_ready_.end(), [](bool b){ return b; }))
    {
      bakePathOnImages(frame_cache_);  // draws path in-place

      for (int i = 0; i < num_cameras_; ++i)
      {
        cv::imshow("Camera " + std::to_string(i), frame_cache_[i]);
        frame_ready_[i] = false;  // reset for next round
      }
      cv::waitKey(1);
    }

  }
  catch (const std::exception & e)
  {
    RCLCPP_ERROR(this->get_logger(), "Error decoding image: %s", e.what());
  }
}

void MultiCameraSubscriber::bakePathOnImages(std::vector<cv::Mat>& images, float path_width)
{
  if (waypoints_.size() < 2)
    return;

  if (!getCurrentRobotPose())
    return;

  const auto& pos = current_pose_.pose.pose.position;
  const auto& ori = current_pose_.pose.pose.orientation;

  // world→body: R_body = R_world^T, t_body = -R_world^T * t_world
  const double qw = ori.w, qx = ori.x, qy = ori.y, qz = ori.z;

  // Body→world rotation
  cv::Mat R_world_cv = (cv::Mat_<double>(3,3) << 1 - 2*(qy*qy + qz*qz), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw),
                                                2*(qx*qy + qz*qw), 1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw),
                                                2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx*qx + qy*qy));

  cv::Mat t_world_cv = (cv::Mat_<double>(3,1) << pos.x, pos.y, pos.z);

  // World → body (LiDAR frame)
  cv::Mat R_body_cv = R_world_cv.t();
  cv::Mat t_body_cv = -R_body_cv * t_world_cv;

  // Find closest waypoint (XY only) and slice 20 ahead
  double min_dist = std::numeric_limits<double>::max();
  size_t closest_idx = 0;

  for (size_t i = 0; i < waypoints_.size(); ++i)
  {
    double dx = waypoints_[i](0) - pos.x;
    double dy = waypoints_[i](1) - pos.y;
    double dist = std::sqrt(dx*dx + dy*dy);
    if (dist < min_dist)
    {
      min_dist  = dist;
      closest_idx = i;
    }
  }

  const size_t start = closest_idx;
  const size_t end   = std::min(closest_idx + 10, waypoints_.size());
  const size_t N     = end - start;
  if (N < 2) return;

  // Build ribbon edges in world frame
  const float half_w = path_width * 0.5f;

  std::vector<cv::Mat> left_world, right_world;   // each is 3x1 double, world frame
  left_world.reserve(N);
  right_world.reserve(N);

  cv::Mat prev_right_cv = cv::Mat::zeros(3, 1, CV_64F);

  for (size_t i = 0; i < N; ++i)
  {
    size_t gi = start + i;  // index into waypoints_

    // Forward direction in XY
    Eigen::Vector3d fwd_e;
    if (i == 0)
      fwd_e = waypoints_[gi + 1].head<3>() - waypoints_[gi].head<3>();
    else if (i == N - 1)
      fwd_e = waypoints_[gi].head<3>() - waypoints_[gi - 1].head<3>();
    else
      fwd_e = waypoints_[gi + 1].head<3>() - waypoints_[gi - 1].head<3>();

    double n = fwd_e.norm();
    if (n < 1e-8) continue;
    fwd_e /= n;

    cv::Mat right_cv = (cv::Mat_<double>(3,1) << -fwd_e.y(), fwd_e.x(), 0.0);
    double rn = cv::norm(right_cv);
    if (rn < 1e-8) right_cv = (cv::Mat_<double>(3,1) << 1.0, 0.0, 0.0);
    else           right_cv /= rn;

    // Flip check to prevent ribbon twist
    if (cv::norm(prev_right_cv) > 0.5 && prev_right_cv.dot(right_cv) < 0.0)
      right_cv = -right_cv;
    prev_right_cv = right_cv.clone();

    // Waypoint position — use waypoint Z directly (map frame ground Z)
    cv::Mat p_cv = (cv::Mat_<double>(3,1) << waypoints_[gi](0), waypoints_[gi](1), waypoints_[gi](2));

    left_world.push_back(p_cv - right_cv * half_w);
    right_world.push_back(p_cv + right_cv * half_w);
  }

  const size_t M = left_world.size();
  if (M < 2) return;

  // Project onto each camera
  // world pt  --(R_body, t_body)-->  LiDAR body pt  --(R_cam, t_cam)-->  camera pt  -->  pixel
  const cv::Scalar path_color(51, 204, 255);
  constexpr double eps = 0.05;

  for (int cam_idx = 0; cam_idx < static_cast<int>(images.size()); ++cam_idx)
  {
    cv::Mat& img = images[cam_idx];
    if (img.empty()) continue;

    if (cam_idx >= static_cast<int>(calib.extrinsics_array.size()) ||
        cam_idx >= static_cast<int>(calib.camera_array.size()))
      continue;

    auto* ext = calib.extrinsics_array[cam_idx].get();
    auto* cam = calib.camera_array[cam_idx].get();
    if (!ext || !cam) continue;

    cv::Mat R_cam = ext->get_R_opencv();   // LiDAR → camera rotation
    cv::Mat t_cam = ext->get_t_opencv();   // LiDAR → camera translation
    cv::Mat K     = cam->get_K();

    const double fx = K.at<double>(0, 0), fy = K.at<double>(1, 1);
    const double cx = K.at<double>(0, 2), cy = K.at<double>(1, 2);
    const int    w  = img.cols,            h  = img.rows;

    // Helper: world 3x1 cv::Mat → pixel, returns false if behind camera
    auto worldToPixel = [&](const cv::Mat& pt_world, cv::Point& px) -> bool
    {
      // Step 1: world → LiDAR body
      cv::Mat pt_lidar = R_body_cv * pt_world + t_body_cv;

      // Step 2: LiDAR body → camera
      cv::Mat pt_cam = R_cam * pt_lidar + t_cam;

      double Z = pt_cam.at<double>(2);
      if (Z <= eps) return false;

      double X = pt_cam.at<double>(0);
      double Y = pt_cam.at<double>(1);

      px.x = static_cast<int>(std::round(fx * X / Z + cx));
      px.y = static_cast<int>(std::round(fy * Y / Z + cy));
      return true;
    };

    cv::Mat overlay = img.clone();
    bool drawn = false;

    for (size_t i = 0; i + 1 < M; ++i)
    {
      cv::Point p0l, p0r, p1l, p1r;
      if (!worldToPixel(left_world[i],      p0l)) continue;
      if (!worldToPixel(right_world[i],     p0r)) continue;
      if (!worldToPixel(left_world[i + 1],  p1l)) continue;
      if (!worldToPixel(right_world[i + 1], p1r)) continue;

      // Optional: skip quads wildly outside image bounds
      auto inBounds = [&](const cv::Point& p) {
        return p.x > -w && p.x < 2*w && p.y > -h && p.y < 2*h;
      };
      if (!inBounds(p0l) || !inBounds(p0r) || !inBounds(p1l) || !inBounds(p1r))
        continue;

      std::vector<cv::Point> quad = { p0l, p0r, p1r, p1l };
      cv::fillConvexPoly(overlay, quad, path_color, cv::LINE_AA);
      drawn = true;
    }

    if (drawn)
      cv::addWeighted(overlay, 0.5, img, 0.5, 0, img);
  }
}


}  // namespace path_processing

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<path_processing::MultiCameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}
