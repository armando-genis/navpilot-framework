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

#include <functional>
#include <memory>
#include <string>
#include <vector>

class MultiCameraSync : public rclcpp::Node
{
public:
  using ImageMsg = sensor_msgs::msg::Image;
  using CloudMsg = sensor_msgs::msg::PointCloud2;

  explicit MultiCameraSync(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : rclcpp::Node("lidar_camera_matcher", opts)
  {
    // Params
    this->declare_parameter<std::vector<std::string>>("cameras", std::vector<std::string>{"/camera_0", "/camera_1"});
    this->declare_parameter<std::string>("sync_suffix", "_sync");
    this->declare_parameter<int>("queue_size", 10);
    this->declare_parameter<bool>("use_lidar", false);
    this->declare_parameter<std::string>("lidar_topic_in", "/lidar/points");
    this->declare_parameter<std::string>("lidar_topic_out", "/lidar/points_sync");

    this->get_parameter("cameras", cameras_);
    this->get_parameter("sync_suffix", sync_suffix_);
    this->get_parameter("queue_size", queue_size_);
    this->get_parameter("use_lidar", use_lidar_);
    this->get_parameter("lidar_topic_in", lidar_topic_in_);
    this->get_parameter("lidar_topic_out", lidar_topic_out_);

    num_cams_ = static_cast<int>(cameras_.size());
    if (num_cams_ <= 0) throw std::runtime_error("cameras list must not be empty");
    if (num_cams_ > 5) throw std::runtime_error("This implementation supports 1..5 cameras");

    auto qos = rclcpp::SensorDataQoS();
    qos_ = qos;
    qos_camera_ = qos;
    rmw_qos_ = qos.get_rmw_qos_profile();
    rmw_qos_camera_ = qos.get_rmw_qos_profile();

    init_pubs();
    init_subs_and_sync();

    RCLCPP_INFO(get_logger(), "Ready: num_cams=%d, use_lidar=%s", num_cams_, use_lidar_ ? "true" : "false");
    for (const auto& cam : cameras_) {
      RCLCPP_INFO(get_logger(), "  - %s", cam.c_str());
    }
  }

private:

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
          cameras_[0], qos_camera_,
          [this](const ImageMsg::ConstSharedPtr msg) {
            image_pubs_[0]->publish(*msg);
          });
        RCLCPP_INFO(get_logger(), "SUB single: %s", cameras_[0].c_str());
        return;
      }

      // Con lidar: usa message_filters para (Image, Cloud)
      sub_single_mf_ = std::make_unique<message_filters::Subscriber<ImageMsg>>(
        this, cameras_[0], rmw_qos_camera_);
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
      cam_subs_.emplace_back(std::make_unique<message_filters::Subscriber<ImageMsg>>(this, cameras_[i], rmw_qos_camera_));
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
  }

  // ---------------- Callback functions ----------------
  void callback_cam1_lidar(const ImageMsg::ConstSharedPtr& img0, const CloudMsg::ConstSharedPtr& cloud)
  {
    image_pubs_[0]->publish(*img0);
    lidar_pub_->publish(*cloud);
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

  // QoS
  rclcpp::QoS qos_{10};
  rmw_qos_profile_t rmw_qos_{rmw_qos_profile_default};
  rclcpp::QoS qos_camera_{10};
  rmw_qos_profile_t rmw_qos_camera_{rmw_qos_profile_default};

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