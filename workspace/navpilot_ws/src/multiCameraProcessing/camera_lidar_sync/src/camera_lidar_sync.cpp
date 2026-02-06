/*
@description:
This node synchronizes camera_msg from different cameras based on their timestamps using message_filters.
It subscribes to multiple camera topics, synchronizes the incoming messages, and republishes them on
designated output topics.

In the Case of only one camera, it simply republishes the incoming messages without synchronization.
Is recommended to use when multiple cameras are used in the same application. use_image_transport_ = false;
*/

#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

#include <filesystem>
#include <fstream>
#include <functional>
#include <iomanip>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

class MultiCameraSync : public rclcpp::Node
{
public:
  using ImageMsg = sensor_msgs::msg::Image;
  using CloudMsg = sensor_msgs::msg::PointCloud2;

  explicit MultiCameraSync(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : rclcpp::Node("multi_camera_sync", opts)
  {
    // Params
    this->declare_parameter<std::vector<std::string>>("cameras", std::vector<std::string>{"/camera_0", "/camera_1"});
    this->declare_parameter<std::string>("sync_suffix", "_sync");
    this->declare_parameter<int>("queue_size", 10);

    this->declare_parameter<bool>("use_lidar", false);
    this->declare_parameter<std::string>("lidar_topic_in", "/lidar/points");
    this->declare_parameter<std::string>("lidar_topic_out", "/lidar/points_sync");

    // Image saving params
    this->declare_parameter<bool>("save_images", false);
    this->declare_parameter<std::string>("output_dir", "/tmp/camera_sync");
    this->declare_parameter<int>("save_interval", 5);
    this->declare_parameter<std::string>("image_encoding", "");  // e.g., "mjpeg", "rgb8", "" for auto-detect

    this->get_parameter("cameras", cameras_);
    this->get_parameter("sync_suffix", sync_suffix_);
    this->get_parameter("queue_size", queue_size_);

    this->get_parameter("use_lidar", use_lidar_);
    this->get_parameter("lidar_topic_in", lidar_topic_in_);
    this->get_parameter("lidar_topic_out", lidar_topic_out_);

    this->get_parameter("save_images", save_images_);
    this->get_parameter("output_dir", output_dir_);
    this->get_parameter("save_interval", save_interval_);
    this->get_parameter("image_encoding", image_encoding_);

    num_cams_ = static_cast<int>(cameras_.size());
    if (num_cams_ <= 0) throw std::runtime_error("cameras list must not be empty");
    if (num_cams_ > 5) throw std::runtime_error("This implementation supports 1..5 cameras");

    qos_ = rclcpp::SensorDataQoS();
    rmw_qos_ = qos_.get_rmw_qos_profile();

    // Create output directories if saving images
    if (save_images_) {
      init_output_dirs();
    }

    init_pubs();
    init_subs_and_sync();

    RCLCPP_INFO(get_logger(), "Ready: num_cams=%d, use_lidar=%s, save_images=%s",
                num_cams_, use_lidar_ ? "true" : "false", save_images_ ? "true" : "false");
    for (const auto& cam : cameras_) {
      RCLCPP_INFO(get_logger(), "  - %s", cam.c_str());
    }
    if (save_images_) {
      RCLCPP_INFO(get_logger(), "Saving every %d frames to: %s", save_interval_, output_dir_.c_str());
    }
  }

private:
  // ---------------- Output directories ----------------
  void init_output_dirs()
  {
    individual_dir_ = output_dir_ + "/individual";
    combined_dir_ = output_dir_ + "/combined";

    std::filesystem::create_directories(individual_dir_);
    std::filesystem::create_directories(combined_dir_);

    RCLCPP_INFO(get_logger(), "Created output dirs: %s, %s",
                individual_dir_.c_str(), combined_dir_.c_str());
  }

  // ---------------- Image saving helpers ----------------
  std::string topic_to_filename(const std::string& topic)
  {
    // Convert topic like "/camera_1" or "/raw/camera_front" to "camera_1" or "raw_camera_front"
    std::string name = topic;
    // Remove leading slashes
    while (!name.empty() && name[0] == '/') {
      name = name.substr(1);
    }
    // Replace remaining slashes with underscores
    std::replace(name.begin(), name.end(), '/', '_');
    return name;
  }

  std::string format_frame_number(int num)
  {
    std::ostringstream oss;
    oss << std::setw(5) << std::setfill('0') << num;
    return oss.str();
  }

  bool is_jpeg_data(const ImageMsg::ConstSharedPtr& msg)
  {
    // First check if encoding is specified in config
    if (!image_encoding_.empty()) {
      return (image_encoding_ == "mjpeg" || image_encoding_ == "jpeg" ||
              image_encoding_ == "MJPEG" || image_encoding_ == "JPEG");
    }
    // Otherwise check message encoding string
    if (msg->encoding == "jpeg" || msg->encoding == "mjpeg" ||
        msg->encoding == "JPEG" || msg->encoding == "MJPEG") {
      return true;
    }
    // Check JPEG magic bytes (FFD8FF) as fallback
    if (msg->data.size() >= 3 &&
        msg->data[0] == 0xFF && msg->data[1] == 0xD8 && msg->data[2] == 0xFF) {
      return true;
    }
    return false;
  }

  // Save raw JPEG data directly to file (fastest - no decode/encode)
  bool save_jpeg_direct(const ImageMsg::ConstSharedPtr& msg, const std::string& filename)
  {
    std::ofstream file(filename, std::ios::binary);
    if (!file.is_open()) {
      RCLCPP_ERROR(get_logger(), "Failed to open file: %s", filename.c_str());
      return false;
    }
    file.write(reinterpret_cast<const char*>(msg->data.data()), msg->data.size());
    file.close();
    return true;
  }

  // Decode JPEG data to cv::Mat (only needed for combined image)
  cv::Mat decode_jpeg(const ImageMsg::ConstSharedPtr& msg)
  {
    cv::Mat raw_data(1, msg->data.size(), CV_8UC1, const_cast<uint8_t*>(msg->data.data()));
    cv::Mat decoded = cv::imdecode(raw_data, cv::IMREAD_COLOR);
    return decoded;
  }

  // Convert non-JPEG ROS image to cv::Mat using cv_bridge
  cv::Mat ros_to_cv(const ImageMsg::ConstSharedPtr& msg)
  {
    try {
      cv_bridge::CvImageConstPtr cv_ptr = cv_bridge::toCvShare(msg);
      cv::Mat img;
      if (cv_ptr->image.channels() == 1) {
        cv::cvtColor(cv_ptr->image, img, cv::COLOR_GRAY2BGR);
      } else if (cv_ptr->encoding == "rgb8") {
        cv::cvtColor(cv_ptr->image, img, cv::COLOR_RGB2BGR);
      } else {
        img = cv_ptr->image.clone();
      }
      return img;
    } catch (const cv_bridge::Exception& e) {
      RCLCPP_ERROR(get_logger(), "cv_bridge exception: %s", e.what());
      return cv::Mat();
    }
  }

  cv::Mat create_combined_image(const std::vector<cv::Mat>& images)
  {
    if (images.empty()) return cv::Mat();

    // Find max dimensions
    int max_height = 0;
    int total_width = 0;
    for (const auto& img : images) {
      if (!img.empty()) {
        max_height = std::max(max_height, img.rows);
        total_width += img.cols;
      }
    }

    if (max_height == 0 || total_width == 0) return cv::Mat();

    // Create combined image (horizontal concatenation)
    cv::Mat combined(max_height, total_width, CV_8UC3, cv::Scalar(0, 0, 0));

    int x_offset = 0;
    for (const auto& img : images) {
      if (!img.empty()) {
        // Center vertically if heights differ
        int y_offset = (max_height - img.rows) / 2;
        img.copyTo(combined(cv::Rect(x_offset, y_offset, img.cols, img.rows)));
        x_offset += img.cols;
      }
    }

    return combined;
  }

  void save_synchronized_images(const std::vector<ImageMsg::ConstSharedPtr>& imgs)
  {
    if (!save_images_) return;

    frame_count_++;
    if (frame_count_ % save_interval_ != 0) return;

    int save_num = frame_count_ / save_interval_;
    std::string frame_str = format_frame_number(save_num);

    std::vector<cv::Mat> cv_images;
    cv_images.reserve(imgs.size());

    // Save individual images
    for (size_t i = 0; i < imgs.size(); ++i) {
      std::string filename = individual_dir_ + "/" + topic_to_filename(cameras_[i]) + "_" + frame_str + ".jpg";

      if (is_jpeg_data(imgs[i])) {
        // Fast path: save raw JPEG directly (no decode/encode)
        save_jpeg_direct(imgs[i], filename);
        // Decode only for combined image
        cv::Mat decoded = decode_jpeg(imgs[i]);
        if (!decoded.empty()) {
          cv_images.push_back(decoded);
        }
      } else {
        // Slow path: convert via cv_bridge and encode
        cv::Mat cv_img = ros_to_cv(imgs[i]);
        if (!cv_img.empty()) {
          cv_images.push_back(cv_img);
          cv::imwrite(filename, cv_img);
        }
      }
    }

    // Save combined image
    cv::Mat combined = create_combined_image(cv_images);
    if (!combined.empty()) {
      std::string filename = combined_dir_ + "/combined_" + frame_str + ".jpg";
      cv::imwrite(filename, combined);
    }

    RCLCPP_DEBUG(get_logger(), "Saved frame %d", save_num);
  }

  // ---------------- Publishers ----------------
  void init_pubs()
  {
    image_pubs_.clear();
    image_pubs_.reserve(num_cams_);

    for (int i = 0; i < num_cams_; ++i) {
      const std::string out_topic = cameras_[i] + sync_suffix_;
      image_pubs_.push_back(this->create_publisher<ImageMsg>(out_topic, qos_));
      RCLCPP_INFO(get_logger(), "PUB: %s -> %s", cameras_[i].c_str(), out_topic.c_str());
    }

    if (use_lidar_) {
      lidar_pub_ = this->create_publisher<CloudMsg>(lidar_topic_out_, qos_);
      RCLCPP_INFO(get_logger(), "PUB lidar: %s", lidar_topic_out_.c_str());
    }
  }

  // ---------------- Subscribers + Sync ----------------
  void init_subs_and_sync()
  {
    // LiDAR subscriber (solo si bandera activada)
    if (use_lidar_) {
      lidar_sub_ = std::make_unique<message_filters::Subscriber<CloudMsg>>(
        this, lidar_topic_in_, rmw_qos_);
      RCLCPP_INFO(get_logger(), "SUB lidar: %s", lidar_topic_in_.c_str());
    }

    // Caso 1 cámara:
    if (num_cams_ == 1) {
      if (!use_lidar_) {
        // Sin lidar: republish directo (sin message_filters)
        sub_single_ros_ = this->create_subscription<ImageMsg>(
          cameras_[0], qos_,
          [this](const ImageMsg::ConstSharedPtr msg) {
            image_pubs_[0]->publish(*msg);
            save_synchronized_images({msg});
          });
        RCLCPP_INFO(get_logger(), "SUB single: %s", cameras_[0].c_str());
        return;
      }

      // Con lidar: usa message_filters para (Image, Cloud)
      sub_single_mf_ = std::make_unique<message_filters::Subscriber<ImageMsg>>(
        this, cameras_[0], rmw_qos_);
      RCLCPP_INFO(get_logger(), "SUB: %s", cameras_[0].c_str());

      using PolicyCam1L = message_filters::sync_policies::ApproximateTime<ImageMsg, CloudMsg>;
      using SyncCam1L   = message_filters::Synchronizer<PolicyCam1L>;

      sync_cam1_lidar_ = std::make_unique<SyncCam1L>(PolicyCam1L(queue_size_), *sub_single_mf_, *lidar_sub_);
      sync_cam1_lidar_->registerCallback(
        std::bind(&MultiCameraSync::callback_cam1_lidar, this, std::placeholders::_1, std::placeholders::_2));
      return;
    }

    // Multi-cam (2..5): message_filters subscribers
    cam_subs_.clear();
    cam_subs_.reserve(num_cams_);

    for (int i = 0; i < num_cams_; ++i) {
      cam_subs_.emplace_back(std::make_unique<message_filters::Subscriber<ImageMsg>>(this, cameras_[i], rmw_qos_));
      RCLCPP_INFO(get_logger(), "SUB: %s", cameras_[i].c_str());
    }

    // Selección de sincronizador según use_lidar + num_cams
    if (!use_lidar_) {
      if      (num_cams_ == 2) init_sync_cam2();
      else if (num_cams_ == 3) init_sync_cam3();
      else if (num_cams_ == 4) init_sync_cam4();
      else if (num_cams_ == 5) init_sync_cam5();
    } else {
      if      (num_cams_ == 2) init_sync_cam2_lidar();
      else if (num_cams_ == 3) init_sync_cam3_lidar();
      else if (num_cams_ == 4) init_sync_cam4_lidar();
      else if (num_cams_ == 5) init_sync_cam5_lidar();
    }
  }

  // ---------------- Publish helpers ----------------
  void publish_images(const std::vector<ImageMsg::ConstSharedPtr>& imgs)
  {
    for (size_t i = 0; i < imgs.size(); ++i) {
      image_pubs_[i]->publish(*imgs[i]);
    }
    save_synchronized_images(imgs);
  }

  // ---------------- Callback functions ----------------
  void callback_cam1_lidar(const ImageMsg::ConstSharedPtr& img0, const CloudMsg::ConstSharedPtr& cloud)
  {
    image_pubs_[0]->publish(*img0);
    lidar_pub_->publish(*cloud);
    save_synchronized_images({img0});
  }

  void callback_cam2(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b)
  {
    publish_images({a, b});
  }

  void callback_cam3(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b,
                     const ImageMsg::ConstSharedPtr& c)
  {
    publish_images({a, b, c});
  }

  void callback_cam4(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b,
                     const ImageMsg::ConstSharedPtr& c, const ImageMsg::ConstSharedPtr& d)
  {
    publish_images({a, b, c, d});
  }

  void callback_cam5(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b,
                     const ImageMsg::ConstSharedPtr& c, const ImageMsg::ConstSharedPtr& d,
                     const ImageMsg::ConstSharedPtr& e)
  {
    publish_images({a, b, c, d, e});
  }

  void callback_cam2_lidar(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b,
                           const CloudMsg::ConstSharedPtr& cloud)
  {
    publish_images({a, b});
    lidar_pub_->publish(*cloud);
  }

  void callback_cam3_lidar(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b,
                           const ImageMsg::ConstSharedPtr& c, const CloudMsg::ConstSharedPtr& cloud)
  {
    publish_images({a, b, c});
    lidar_pub_->publish(*cloud);
  }

  void callback_cam4_lidar(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b,
                           const ImageMsg::ConstSharedPtr& c, const ImageMsg::ConstSharedPtr& d,
                           const CloudMsg::ConstSharedPtr& cloud)
  {
    publish_images({a, b, c, d});
    lidar_pub_->publish(*cloud);
  }

  void callback_cam5_lidar(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b,
                           const ImageMsg::ConstSharedPtr& c, const ImageMsg::ConstSharedPtr& d,
                           const ImageMsg::ConstSharedPtr& e, const CloudMsg::ConstSharedPtr& cloud)
  {
    publish_images({a, b, c, d, e});
    lidar_pub_->publish(*cloud);
  }

  // ---------------- Camera-only synchronizers ----------------
  using Policy2 = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
  using Sync2   = message_filters::Synchronizer<Policy2>;
  std::unique_ptr<Sync2> sync2_;

  void init_sync_cam2()
  {
    sync2_ = std::make_unique<Sync2>(Policy2(queue_size_), *cam_subs_[0], *cam_subs_[1]);
    sync2_->registerCallback(std::bind(&MultiCameraSync::callback_cam2, this,
      std::placeholders::_1, std::placeholders::_2));
  }

  using Policy3 = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg>;
  using Sync3   = message_filters::Synchronizer<Policy3>;
  std::unique_ptr<Sync3> sync3_;

  void init_sync_cam3()
  {
    sync3_ = std::make_unique<Sync3>(Policy3(queue_size_), *cam_subs_[0], *cam_subs_[1], *cam_subs_[2]);
    sync3_->registerCallback(std::bind(&MultiCameraSync::callback_cam3, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  using Policy4 = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg>;
  using Sync4   = message_filters::Synchronizer<Policy4>;
  std::unique_ptr<Sync4> sync4_;

  void init_sync_cam4()
  {
    sync4_ = std::make_unique<Sync4>(Policy4(queue_size_), *cam_subs_[0], *cam_subs_[1], *cam_subs_[2], *cam_subs_[3]);
    sync4_->registerCallback(std::bind(&MultiCameraSync::callback_cam4, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }

  using Policy5 = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg>;
  using Sync5   = message_filters::Synchronizer<Policy5>;
  std::unique_ptr<Sync5> sync5_;

  void init_sync_cam5()
  {
    sync5_ = std::make_unique<Sync5>(Policy5(queue_size_), *cam_subs_[0], *cam_subs_[1], *cam_subs_[2], *cam_subs_[3], *cam_subs_[4]);
    sync5_->registerCallback(std::bind(&MultiCameraSync::callback_cam5, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
  }

  // ---------------- Camera + LiDAR synchronizers ----------------
  using Policy2L = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, CloudMsg>;
  using Sync2L   = message_filters::Synchronizer<Policy2L>;
  std::unique_ptr<Sync2L> sync2_lidar_;

  void init_sync_cam2_lidar()
  {
    sync2_lidar_ = std::make_unique<Sync2L>(Policy2L(queue_size_), *cam_subs_[0], *cam_subs_[1], *lidar_sub_);
    sync2_lidar_->registerCallback(std::bind(&MultiCameraSync::callback_cam2_lidar, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  using Policy3L = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, CloudMsg>;
  using Sync3L   = message_filters::Synchronizer<Policy3L>;
  std::unique_ptr<Sync3L> sync3_lidar_;

  void init_sync_cam3_lidar()
  {
    sync3_lidar_ = std::make_unique<Sync3L>(Policy3L(queue_size_), *cam_subs_[0], *cam_subs_[1], *cam_subs_[2], *lidar_sub_);
    sync3_lidar_->registerCallback(std::bind(&MultiCameraSync::callback_cam3_lidar, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }

  using Policy4L = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, CloudMsg>;
  using Sync4L   = message_filters::Synchronizer<Policy4L>;
  std::unique_ptr<Sync4L> sync4_lidar_;

  void init_sync_cam4_lidar()
  {
    sync4_lidar_ = std::make_unique<Sync4L>(Policy4L(queue_size_), *cam_subs_[0], *cam_subs_[1], *cam_subs_[2], *cam_subs_[3], *lidar_sub_);
    sync4_lidar_->registerCallback(std::bind(&MultiCameraSync::callback_cam4_lidar, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
  }

  using Policy5L = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, CloudMsg>;
  using Sync5L   = message_filters::Synchronizer<Policy5L>;
  std::unique_ptr<Sync5L> sync5_lidar_;

  void init_sync_cam5_lidar()
  {
    sync5_lidar_ = std::make_unique<Sync5L>(Policy5L(queue_size_), *cam_subs_[0], *cam_subs_[1], *cam_subs_[2], *cam_subs_[3], *cam_subs_[4], *lidar_sub_);
    sync5_lidar_->registerCallback(std::bind(&MultiCameraSync::callback_cam5_lidar, this,
      std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
  }

private:
  // params
  std::vector<std::string> cameras_;
  std::string sync_suffix_;
  int queue_size_{10};
  int num_cams_{0};

  // lidar params
  bool use_lidar_{false};
  std::string lidar_topic_in_;
  std::string lidar_topic_out_;

  // image saving params
  bool save_images_{false};
  std::string output_dir_;
  int save_interval_{5};
  std::string image_encoding_;  // e.g., "mjpeg", "rgb8", "" for auto-detect
  std::string individual_dir_;
  std::string combined_dir_;
  int frame_count_{0};

  // QoS
  rclcpp::QoS qos_{10};
  rmw_qos_profile_t rmw_qos_{rmw_qos_profile_default};

  // camera pubs
  std::vector<rclcpp::Publisher<ImageMsg>::SharedPtr> image_pubs_;

  // single-cam (no lidar) fast path
  rclcpp::Subscription<ImageMsg>::SharedPtr sub_single_ros_;

  // single-cam (with lidar) mf sub + sync
  std::unique_ptr<message_filters::Subscriber<ImageMsg>> sub_single_mf_;
  using PolicyCam1L = message_filters::sync_policies::ApproximateTime<ImageMsg, CloudMsg>;
  using SyncCam1L   = message_filters::Synchronizer<PolicyCam1L>;
  std::unique_ptr<SyncCam1L> sync_cam1_lidar_;

  // multi-cam mf subs
  std::vector<std::unique_ptr<message_filters::Subscriber<ImageMsg>>> cam_subs_;

  // lidar sub/pub
  std::unique_ptr<message_filters::Subscriber<CloudMsg>> lidar_sub_;
  rclcpp::Publisher<CloudMsg>::SharedPtr lidar_pub_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiCameraSync>());
  rclcpp::shutdown();
  return 0;
}