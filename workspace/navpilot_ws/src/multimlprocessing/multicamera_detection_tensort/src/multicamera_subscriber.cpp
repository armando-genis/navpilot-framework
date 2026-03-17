#include "multicamera_subscriber.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <iomanip>

namespace multicamera_detection_tensort
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

MultiCameraSubscriber::MultiCameraSubscriber() : Node("multicamera_detection_tensort_node")
{
  this->declare_parameter<int>("num_cameras", 3);
  this->declare_parameter<std::string>("calib_dir", "");
  this->declare_parameter<std::string>("model_path", "");
  this->declare_parameter<bool>("use_gpu", false);
  this->declare_parameter<bool>("enable_yaw", false);
  this->declare_parameter<std::string>("lidar_topic", "");

  num_cameras_ = this->get_parameter("num_cameras").as_int();
  calib_dir_   = this->get_parameter("calib_dir").as_string();
  model_path_  = this->get_parameter("model_path").as_string();
  use_gpu_     = this->get_parameter("use_gpu").as_bool();
  enable_yaw_  = this->get_parameter("enable_yaw").as_bool();
  lidar_topic_ = this->get_parameter("lidar_topic").as_string();

  // ── YoloV8 init ────────────────────────────────────────────────────────────
  YoloV8Config config;
  config.probabilityThreshold = _CONF_THR;  // 0.3f
  config.nmsThreshold         = 0.65f;
  config.numKPS               = 17;         // COCO pose keypoints
  config.kpsThreshold         = _CONF_THR;
  config.classNames           = {"person"}; // pose model only detects people

  // trt engine is auto-built from onnx on first run (~5 min), then cached
  std::string trt_path = model_path_ + ".engine";
  yolo_ = std::make_unique<YoloV8>(model_path_, trt_path, config);
  RCLCPP_INFO(this->get_logger(), "YoloV8 loaded: %s  →  engine: %s",
      model_path_.c_str(), trt_path.c_str());


  calib.load(calib_dir_);

  lidar_cloud_.reset(new pcl::PointCloud<pcl::PointXYZI>());

  auto qos = rclcpp::SensorDataQoS();

  // Subscribe to lidar topic
  lidar_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      lidar_topic_,
      qos,
      std::bind(&MultiCameraSubscriber::lidarCallback, this, std::placeholders::_1)
  );

  // Subscribe to camera topics
  for (int i = 0; i < num_cameras_; i++)
  {
    std::string topic = "/racecar/camera/camera_" + std::to_string(i) + "/image_raw_sync";
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

  auto pub_qos = rclcpp::QoS(rclcpp::KeepLast(1)).reliable();
  marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      "/detected_persons_markers", pub_qos);

  cv::namedWindow("MultiCamera", cv::WINDOW_NORMAL);

  infer_thread_ = std::thread(&MultiCameraSubscriber::inferenceLoop, this);
}

MultiCameraSubscriber::~MultiCameraSubscriber()
{
  {
    std::lock_guard<std::mutex> lock(queue_mutex_);
    running_ = false;
  }
  queue_cv_.notify_all();
  if (infer_thread_.joinable())
    infer_thread_.join();
}

void MultiCameraSubscriber::imageCallback(
    sensor_msgs::msg::Image::ConstSharedPtr msg, int camera_id)
{
  try
  {
    cv::Mat frame;
    if (msg->encoding == "mjpeg") {
      std::vector<uint8_t> buf(msg->data.begin(), msg->data.end());
      frame = cv::imdecode(buf, cv::IMREAD_COLOR);
    } else {
      RCLCPP_WARN_ONCE(this->get_logger(), "Unsupported encoding: %s", msg->encoding.c_str());
      return;
    }
    if (frame.empty()) return;

    // Undistort (fast — no inference here)
    if (static_cast<size_t>(camera_id) < calib.camera_array.size()) {
      auto& cam = calib.camera_array[camera_id];
      if (cam) { cam->ensure_size(frame.cols, frame.rows); frame = cam->undistort(frame); }
    }

    // Snapshot latest cloud — non-blocking
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_snap;
    {
      std::lock_guard<std::mutex> lock(lidar_mutex_);
      if (lidar_cloud_ && !lidar_cloud_->empty())
        cloud_snap = std::make_shared<pcl::PointCloud<pcl::PointXYZI>>(*lidar_cloud_);
    }

    // Deposit into queue — drops previous frame if inference is still busy
    {
      std::lock_guard<std::mutex> lock(queue_mutex_);
      auto pf       = std::make_unique<PendingFrame>();
      pf->image     = std::move(frame);
      pf->cloud     = cloud_snap;          // nullptr if lidar not yet received
      pf->camera_id = camera_id;
      pending_frame_ = std::move(pf);      // overwrites any unprocessed frame
    }
    queue_cv_.notify_one();
  }
  catch (const std::exception& e) {
    RCLCPP_ERROR(this->get_logger(), "imageCallback error: %s", e.what());
  }
}

void MultiCameraSubscriber::lidarCallback(
    sensor_msgs::msg::PointCloud2::ConstSharedPtr msg)
{
  if (msg->data.empty()) return;
  std::lock_guard<std::mutex> lock(lidar_mutex_);
  pcl::fromROSMsg(*msg, *lidar_cloud_);
  tree_dirty_ = true;   // signal inference thread to rebuild
}

void MultiCameraSubscriber::inferenceLoop()
{
  while (true)
  {
    std::unique_ptr<PendingFrame> pf;
    {
      std::unique_lock<std::mutex> lock(queue_mutex_);
      queue_cv_.wait(lock, [this] { return pending_frame_ || !running_; });
      if (!running_) break;
      pf = std::move(pending_frame_);
    }

    // ── Rebuild KDTree if lidar cloud updated ─────────────────────────────
    if (tree_dirty_ && pf->cloud) {
      lidar_tree_.build(pf->cloud, calib.extrinsics_array,
                        calib.camera_array, pf->image.cols, pf->image.rows);
      tree_dirty_ = false;
    }

    if (yolo_)
    {
      const int orig_w = pf->image.cols;
      const int orig_h = pf->image.rows;
      float scale  = std::min(640.0f / orig_w, 640.0f / orig_h);
      int scaled_w = static_cast<int>(orig_w * scale);
      int scaled_h = static_cast<int>(orig_h * scale);
      int pad_x    = (640 - scaled_w) / 2;
      int pad_y    = (640 - scaled_h) / 2;

      // Letterbox to 640x640
      cv::Mat letterboxed(640, 640, pf->image.type(), cv::Scalar(114, 114, 114));
      cv::resize(pf->image,
                 letterboxed(cv::Rect(pad_x, pad_y, scaled_w, scaled_h)),
                 cv::Size(scaled_w, scaled_h));

      // ── Run inference ──────────────────────────────────────────────────
      auto detections = yolo_->detectObjects(letterboxed);

      // Draw detections on the letterboxed frame, then scale back
      yolo_->drawObjectLabels(letterboxed, detections);
      cv::resize(letterboxed, pf->image, cv::Size(orig_w, orig_h));

      // ── Extrinsics for floor projection ───────────────────────────────
      cv::Mat R, t, K;
      bool has_calib = false;
      if (static_cast<size_t>(pf->camera_id) < calib.extrinsics_array.size() &&
          static_cast<size_t>(pf->camera_id) < calib.camera_array.size())
      {
        auto& ext = calib.extrinsics_array[pf->camera_id];
        auto& cam = calib.camera_array[pf->camera_id];
        if (ext && cam) {
          R = ext->get_R_opencv();
          t = ext->get_t_opencv();
          K = cam->get_K();
          has_calib = true;
        }
      }

      // ── Publish MarkerArray ───────────────────────────────────────────────
      visualization_msgs::msg::MarkerArray marker_array;

      // First add a DELETE_ALL marker to wipe previous frame's markers
      visualization_msgs::msg::Marker clear_marker;
      clear_marker.header.frame_id = "velodyne";
      clear_marker.header.stamp    = this->now();
      clear_marker.ns              = "detected_persons";
      clear_marker.id              = 0;
      clear_marker.action          = visualization_msgs::msg::Marker::DELETEALL;
      marker_array.markers.push_back(clear_marker);

      // Now add one marker per detection
      int marker_id = 0;
      for (const auto& det : detections)
      {
        float u = (det.rect.x + det.rect.width  * 0.5f - pad_x) / scale;
        float v = (det.rect.y + det.rect.height * 0.5f - pad_y) / scale;

        auto pos3d = lidar_tree_.query(pf->camera_id, u, v);
        if (!pos3d) continue;

        visualization_msgs::msg::Marker marker;
        marker.header.frame_id    = "velodyne";
        marker.header.stamp       = this->now();
        marker.ns                 = "detected_persons";
        marker.id                 = marker_id++;          // 0, 1, 2... per frame
        marker.type               = visualization_msgs::msg::Marker::CYLINDER;
        marker.action             = visualization_msgs::msg::Marker::ADD;

        // Same center the floor circle uses
        marker.pose.position.x    = pos3d->x;
        marker.pose.position.y    = pos3d->y;
        marker.pose.position.z    = pos3d->z + 0.9f;
        marker.pose.orientation.w = 1.0;

        marker.scale.x            = 0.8f;   // diameter = 2 * floor circle radius
        marker.scale.y            = 0.8f;
        marker.scale.z            = 1.8f;

        marker.color.r            = 1.0f;
        marker.color.g            = 0.5f;
        marker.color.b            = 0.0f;
        marker.color.a            = 0.6f;

        marker.lifetime           = rclcpp::Duration::from_seconds(0.5);

        marker_array.markers.push_back(marker);
      }

      marker_pub_->publish(marker_array);  // always publish, even if only DELETEALL

      // image procesing

      for (const auto& det : detections)
      {
        // Map box center: letterboxed 640x640 → original image space
        float u = (det.rect.x + det.rect.width  * 0.5f - pad_x) / scale;
        float v = (det.rect.y + det.rect.height * 0.5f - pad_y) / scale;

        auto pos3d = lidar_tree_.query(pf->camera_id, u, v);
        if (!pos3d) continue;

        // 3D distance label
        cv::putText(pf->image,
            cv::format("%.1f, %.1f m", pos3d->x, pos3d->y),
            cv::Point(static_cast<int>(u) - 40, static_cast<int>(v) - 10),
            cv::FONT_HERSHEY_SIMPLEX, 0.5, {0, 255, 255}, 1, cv::LINE_AA);

        std::optional<float> yaw_angle;
        if (enable_yaw_)
          yaw_angle = computePersonOrientationFromKpts(det, pf->camera_id,
                                                       scale, pad_x, pad_y);

        if (has_calib)
        {
          const float fx_K = static_cast<float>(K.at<double>(0,0));
          const float fy_K = static_cast<float>(K.at<double>(1,1));
          const float cx_K = static_cast<float>(K.at<double>(0,2));
          const float cy_K = static_cast<float>(K.at<double>(1,2));

          auto project = [&](float wx, float wy, float wz) -> std::optional<cv::Point>
          {
            cv::Mat pt     = (cv::Mat_<double>(3,1) << wx, wy, wz);
            cv::Mat cam_pt = R * pt + t;
            double Z = cam_pt.at<double>(2);
            if (Z <= 0.1) return std::nullopt;
            int pu = static_cast<int>(fx_K * cam_pt.at<double>(0) / Z + cx_K + 0.5f);
            int pv = static_cast<int>(fy_K * cam_pt.at<double>(1) / Z + cy_K + 0.5f);
            return cv::Point{pu, pv};
          };

          // Floor circle
          const float px = pos3d->x, py = pos3d->y;
          const float radius = 0.4f;
          const int segments = 24;
          std::vector<cv::Point> ring;
          ring.reserve(segments);
          bool ring_ok = true;

          for (int s = 0; s < segments; s++) {
            float a = 2.0f * static_cast<float>(M_PI) * s / segments;
            auto pt = project(px + radius*std::cos(a), py + radius*std::sin(a), 0.f);
            if (!pt) { ring_ok = false; break; }
            ring.push_back(*pt);
          }

          if (ring_ok && !ring.empty()) {
            cv::Mat overlay = pf->image.clone();
            cv::fillPoly(overlay, std::vector<std::vector<cv::Point>>{ring},
                         cv::Scalar(0, 200, 255));
            cv::addWeighted(overlay, 0.25, pf->image, 0.75, 0, pf->image);
            for (int s = 0; s < segments; s++)
              cv::line(pf->image, ring[s], ring[(s+1)%segments],
                       cv::Scalar(0, 200, 255), 2, cv::LINE_AA);
          }

          if (enable_yaw_ && yaw_angle) {
            auto p_base = project(px, py, 0.f);
            auto p_tip  = project(px + 0.7f*std::cos(*yaw_angle),
                                  py + 0.7f*std::sin(*yaw_angle), 0.f);
            if (p_base && p_tip) {
              cv::arrowedLine(pf->image, *p_base, *p_tip,
                              cv::Scalar(0,80,255), 3, cv::LINE_AA, 0, 0.25);
              cv::putText(pf->image,
                  cv::format("%.0f deg", *yaw_angle * 180.f / M_PI),
                  cv::Point(p_tip->x+5, p_tip->y),
                  cv::FONT_HERSHEY_SIMPLEX, 0.45, {0,80,255}, 1, cv::LINE_AA);
            }
          }
        }
      }
    }

    cv::putText(pf->image, "Camera " + std::to_string(pf->camera_id),
        {20, 40}, cv::FONT_HERSHEY_SIMPLEX, 1.0, {0,255,0}, 2);
    cv::imshow("Camera " + std::to_string(pf->camera_id), pf->image);
    cv::waitKey(1);
  }
}

// Returns yaw angle in radians (in LiDAR frame), or nullopt if not enough data.
// Keypoint indices (COCO): 5=left_shoulder, 6=right_shoulder, 11=left_hip, 12=right_hip
// det.kps is flat: [x0, y0, conf0,  x1, y1, conf1, ...]
// COCO indices: 5=left_shoulder, 6=right_shoulder, 11=left_hip, 12=right_hip, 0=nose
std::optional<float> MultiCameraSubscriber::computePersonOrientationFromKpts(
    const Object& det,
    int camera_id, float scale, int pad_x, int pad_y)
{
  const auto& kps = det.kps;

  auto kpt3d = [&](int idx) -> std::optional<cv::Point3f>
  {
    const int base = idx * 3;
    if (base + 2 >= static_cast<int>(kps.size())) return std::nullopt;
    if (kps[base + 2] < _CONF_THR) return std::nullopt;               // confidence
    float x = (kps[base + 0] - pad_x) / scale;                        // unletterbox x
    float y = (kps[base + 1] - pad_y) / scale;                        // unletterbox y
    return lidar_tree_.query(camera_id, x, y, 0.4f);
  };

  // Try shoulders first, fall back to hips
  auto L = kpt3d(5);  auto R_kpt = kpt3d(6);
  if (!L || !R_kpt) { L = kpt3d(11); R_kpt = kpt3d(12); }
  if (!L || !R_kpt) return std::nullopt;

  // Perpendicular to shoulder/hip vector = facing direction
  float dx = R_kpt->x - L->x,  dy = R_kpt->y - L->y;
  float fx =  dy,  fy = -dx;

  // Disambiguate using nose
  auto nose = kpt3d(0);
  if (nose) {
    cv::Point3f mid((L->x + R_kpt->x) * 0.5f, (L->y + R_kpt->y) * 0.5f, 0.f);
    if ((nose->x - mid.x) * fx + (nose->y - mid.y) * fy < 0.f)
      { fx = -fx; fy = -fy; }
  }

  return std::atan2(fy, fx);
}

cv::Mat MultiCameraSubscriber::projectLidarOnImage(
    const cv::Mat& img,
    const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud,
    const cv::Mat& R,
    const cv::Mat& t,
    const cv::Mat& K)
{
    // Darken the background image
    cv::Mat out;
    img.convertTo(out, -1, 0.35, 0);   // 0.35 = brightness scale, 0 = bias

    if (!cloud || cloud->points.empty())
        return out;

    const double fx = K.at<double>(0, 0);
    const double fy = K.at<double>(1, 1);
    const double cx = K.at<double>(0, 2);
    const double cy = K.at<double>(1, 2);
    const int h = out.rows, w = out.cols;

    // Pre-scan X range for normalization
    float x_min =  std::numeric_limits<float>::max();
    float x_max = -std::numeric_limits<float>::max();
    for (const auto& p : cloud->points) {
        x_min = std::min(x_min, p.x);
        x_max = std::max(x_max, p.x);
    }
    const float x_range = (x_max - x_min) > 1e-3f ? (x_max - x_min) : 1.0f;

    for (const auto& p : cloud->points)
    {
        cv::Mat pt  = (cv::Mat_<double>(3,1) << p.x, p.y, p.z);
        cv::Mat cam = R * pt + t;

        double X = cam.at<double>(0);
        double Y = cam.at<double>(1);
        double Z = cam.at<double>(2);

        if (Z <= 0.1) continue;

        int u = static_cast<int>(fx * X / Z + cx + 0.5);
        int v = static_cast<int>(fy * Y / Z + cy + 0.5);
        if (u < 0 || u >= w || v < 0 || v >= h) continue;

        // Normalize lateral X: 0.0 = leftmost, 1.0 = rightmost
        float nx = (p.x - x_min) / x_range;   // 0..1

        // Color ramp: left=red (255,0,0), center=white (255,255,255), right=blue (0,0,255)
        int r, g, b;
        if (nx < 0.5f) {
            float f = nx * 2.0f;               // 0..1 over left half
            r = 255;
            g = static_cast<int>(255 * f);
            b = static_cast<int>(255 * f);
        } else {
            float f = (nx - 0.5f) * 2.0f;     // 0..1 over right half
            r = static_cast<int>(255 * (1.0f - f));
            g = static_cast<int>(255 * (1.0f - f));
            b = 255;
        }

        cv::circle(out, {u, v}, 2, cv::Scalar(b, g, r), -1);
    }

    return out;
}

}  // namespace multicamera_detection_tensort

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<multicamera_detection_tensort::MultiCameraSubscriber>());
  rclcpp::shutdown();
  return 0;
}
