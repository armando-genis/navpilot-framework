#include <rclcpp/rclcpp.hpp>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <message_filters/pass_through.h>

#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <functional>
#include <memory>
#include <string>
#include <vector>
#include <deque>
#include <mutex>
#include <cmath>
#include <limits>
#include <array>

class MultiCameraSync : public rclcpp::Node
{
public:
  using ImageMsg = sensor_msgs::msg::Image;
  using CloudMsg = sensor_msgs::msg::PointCloud2;

  explicit MultiCameraSync(const rclcpp::NodeOptions& opts = rclcpp::NodeOptions())
  : rclcpp::Node("lidar_camera_matcher", opts)
  {
    this->declare_parameter<std::vector<std::string>>("cameras",
      std::vector<std::string>{"/camera_0", "/camera_1"});
    this->declare_parameter<std::string>("sync_suffix",          "_sync");
    this->declare_parameter<int>("queue_size",                   100);
    this->declare_parameter<bool>("use_lidar",                   false);
    this->declare_parameter<std::string>("lidar_topic_in",       "/lidar/points");
    this->declare_parameter<std::string>("lidar_topic_out",      "/lidar/points_sync");
    this->declare_parameter<double>("cam_min_dt_initial",        0.060);
    this->declare_parameter<double>("lidar_min_dt_initial",      0.085);
    this->declare_parameter<double>("rate_slack",                0.85);
    this->declare_parameter<int>("rate_window",                  20);

    // When true, every incoming message gets its header.stamp replaced
    // with now() BEFORE being fed into ApproximateTime.
    // This is the correct place to restamp — inside the synchronizer
    // input, not after the callback fires.
    this->declare_parameter<bool>("restamp_inputs", true);

    this->get_parameter("cameras",               cameras_);
    this->get_parameter("sync_suffix",           sync_suffix_);
    this->get_parameter("queue_size",            queue_size_);
    this->get_parameter("use_lidar",             use_lidar_);
    this->get_parameter("lidar_topic_in",        lidar_topic_in_);
    this->get_parameter("lidar_topic_out",       lidar_topic_out_);
    this->get_parameter("cam_min_dt_initial",    cam_min_dt_);
    this->get_parameter("lidar_min_dt_initial",  lidar_min_dt_);
    this->get_parameter("rate_slack",            rate_slack_);
    this->get_parameter("rate_window",           rate_window_);
    this->get_parameter("restamp_inputs",        restamp_inputs_);

    num_cams_ = static_cast<int>(cameras_.size());
    if (num_cams_ <= 0) throw std::runtime_error("cameras list must not be empty");
    if (num_cams_ > 5)  throw std::runtime_error("This implementation supports 1..5 cameras");

    auto qos        = rclcpp::SensorDataQoS();
    qos_            = qos;
    qos_camera_     = qos;
    rmw_qos_        = qos.get_rmw_qos_profile();
    rmw_qos_camera_ = qos.get_rmw_qos_profile();

    cam_stamps_.resize(5);
    cam_min_dt_per_.fill(cam_min_dt_);

    // Diagnostic raw subscribers + timer
    diag_lidar_sub_ = this->create_subscription<CloudMsg>(
      lidar_topic_in_, qos_,
      [this](const CloudMsg::ConstSharedPtr msg) { diag_on_lidar(msg); });
    for (int i = 0; i < num_cams_; ++i) {
      diag_cam_subs_.push_back(this->create_subscription<ImageMsg>(
        cameras_[i], qos_camera_,
        [this, i](const ImageMsg::ConstSharedPtr msg) { diag_on_camera(i, msg); }));
    }
    diag_timer_ = this->create_wall_timer(std::chrono::seconds(2),
      [this]() { print_diagnostics(); });

    init_pubs();
    init_subs_and_sync();

    RCLCPP_INFO(get_logger(),
      "Ready: num_cams=%d  use_lidar=%s  queue=%d  restamp=%s  "
      "cam_min_dt=%.3fs  lidar_min_dt=%.3fs  slack=%.2f  window=%d",
      num_cams_, use_lidar_ ? "true" : "false", queue_size_,
      restamp_inputs_ ? "YES (stamps replaced BEFORE sync)" : "NO",
      cam_min_dt_, lidar_min_dt_, rate_slack_, rate_window_);

    for (const auto& cam : cameras_)
      RCLCPP_INFO(get_logger(), "  - %s", cam.c_str());
  }

private:

  // ══════════════════════════════════════════════════════
  // PassThrough filters — one per input topic
  //
  // These sit between the raw ROS subscriber and the
  // ApproximateTime synchronizer. When restamp_inputs is true,
  // we intercept each incoming message, replace its stamp with
  // now(), and feed the modified copy into the synchronizer.
  //
  // This is the CORRECT place to restamp — the synchronizer
  // matches on the stamps it sees at input, so the stamp must
  // be fixed BEFORE the message enters the sync queue.
  // ══════════════════════════════════════════════════════

  // Raw ROS subscribers that intercept, restamp, then signal the PassThrough
  std::vector<rclcpp::Subscription<ImageMsg>::SharedPtr> raw_cam_subs_;
  rclcpp::Subscription<CloudMsg>::SharedPtr              raw_lidar_sub_;

  // PassThrough filters that feed into the synchronizer
  std::vector<std::shared_ptr<message_filters::PassThrough<ImageMsg>>> cam_pass_;
  std::shared_ptr<message_filters::PassThrough<CloudMsg>>              lidar_pass_;

  void setup_restamping_layer()
  {
    // ── Camera PassThrough filters ────────────────────────
    cam_pass_.clear();
    for (int i = 0; i < num_cams_; ++i) {
      auto pass = std::make_shared<message_filters::PassThrough<ImageMsg>>();
      cam_pass_.push_back(pass);

      raw_cam_subs_.push_back(this->create_subscription<ImageMsg>(
        cameras_[i], qos_camera_,
        [this, i, pass](const ImageMsg::ConstSharedPtr msg) {
          if (restamp_inputs_) {
            auto m = std::make_shared<ImageMsg>(*msg);
            m->header.stamp = this->get_clock()->now();
            pass->add(m);
          } else {
            pass->add(std::const_pointer_cast<ImageMsg>(msg));
          }
        }));
    }

    // ── LiDAR PassThrough filter ──────────────────────────
    if (use_lidar_) {
      lidar_pass_ = std::make_shared<message_filters::PassThrough<CloudMsg>>();

      raw_lidar_sub_ = this->create_subscription<CloudMsg>(
        lidar_topic_in_, qos_,
        [this](const CloudMsg::ConstSharedPtr msg) {
          if (restamp_inputs_) {
            auto m = std::make_shared<CloudMsg>(*msg);
            m->header.stamp = this->get_clock()->now();
            lidar_pass_->add(m);
          } else {
            lidar_pass_->add(std::const_pointer_cast<CloudMsg>(msg));
          }
        });
    }
  }

  // ══════════════════════════════════════════════════════
  // Diagnostics (reads ORIGINAL stamps from raw topics)
  // ══════════════════════════════════════════════════════
  std::mutex           diag_mtx_;
  uint64_t             diag_lidar_count_{0};
  rclcpp::Time         diag_lidar_stamp_{0, 0, RCL_ROS_TIME};
  std::array<uint64_t,5>     diag_cam_count_{};
  std::array<rclcpp::Time,5> diag_cam_stamp_{
    rclcpp::Time(0,0,RCL_ROS_TIME), rclcpp::Time(0,0,RCL_ROS_TIME),
    rclcpp::Time(0,0,RCL_ROS_TIME), rclcpp::Time(0,0,RCL_ROS_TIME),
    rclcpp::Time(0,0,RCL_ROS_TIME)};
  uint64_t             diag_sync_count_{0};

  void diag_on_lidar(const CloudMsg::ConstSharedPtr& msg)
  { std::lock_guard<std::mutex> lk(diag_mtx_); ++diag_lidar_count_; diag_lidar_stamp_ = rclcpp::Time(msg->header.stamp); }

  void diag_on_camera(int idx, const ImageMsg::ConstSharedPtr& msg)
  { std::lock_guard<std::mutex> lk(diag_mtx_); ++diag_cam_count_[idx]; diag_cam_stamp_[idx] = rclcpp::Time(msg->header.stamp); }

  void print_diagnostics()
  {
    std::lock_guard<std::mutex> lk(diag_mtx_);
    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
    RCLCPP_INFO(get_logger(), "SYNC DIAGNOSTICS  (sync_callbacks_fired=%lu)", diag_sync_count_);
    for (int i = 0; i < num_cams_; ++i) {
      if (diag_cam_count_[i] == 0)
        RCLCPP_WARN(get_logger(), "  cam[%d] %s → NO MESSAGES RECEIVED", i, cameras_[i].c_str());
      else
        RCLCPP_INFO(get_logger(), "  cam[%d] count=%-6lu  stamp=%.3f s (original)",
          i, diag_cam_count_[i], diag_cam_stamp_[i].seconds());
    }
    if (diag_lidar_count_ == 0)
      RCLCPP_WARN(get_logger(), "  lidar  %s → NO MESSAGES RECEIVED", lidar_topic_in_.c_str());
    else
      RCLCPP_INFO(get_logger(), "  lidar  count=%-6lu  stamp=%.3f s (original)",
        diag_lidar_count_, diag_lidar_stamp_.seconds());

    if (use_lidar_ && diag_lidar_count_ > 0) {
      for (int i = 0; i < num_cams_; ++i) {
        if (diag_cam_count_[i] > 0) {
          double delta = (diag_cam_stamp_[i] - diag_lidar_stamp_).seconds();
          const char* status =
            std::abs(delta) > 0.5  ? "⚠ CLOCK MISMATCH (restamp_inputs handles this)" :
            std::abs(delta) > 0.15 ? "⚠ delta > 150ms" : "✓ OK";
          RCLCPP_INFO(get_logger(),
            "  original stamp_delta cam[%d]-lidar = %+.3f s  %s", i, delta, status);
        }
      }
    }
    RCLCPP_INFO(get_logger(),
      "  restamp_inputs=%s  bounds: cam=%.3fs  lidar=%.3fs",
      restamp_inputs_ ? "YES" : "NO",
      cam_min_dt_ * rate_slack_, lidar_min_dt_ * rate_slack_);
    RCLCPP_INFO(get_logger(), "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━");
  }

  // ══════════════════════════════════════════════════════
  // Runtime min-period estimation (uses now() — wall arrival time)
  // ══════════════════════════════════════════════════════
  double update_min_dt(std::deque<rclcpp::Time>& stamps, double& cur, const rclcpp::Time& t)
  {
    stamps.push_back(t);
    if (static_cast<int>(stamps.size()) > rate_window_ + 1) stamps.pop_front();
    if (static_cast<int>(stamps.size()) < 2) return -1.0;
    double new_min = std::numeric_limits<double>::max();
    for (size_t i = 1; i < stamps.size(); ++i) {
      double dt = (stamps[i] - stamps[i-1]).seconds();
      if (dt > 0.001 && dt < new_min) new_min = dt;
    }
    if (new_min == std::numeric_limits<double>::max()) return -1.0;
    if (std::abs(new_min - cur) / cur < 0.05) return -1.0;
    cur = new_min; return new_min;
  }

  void update_cam_rate(int idx, const rclcpp::Time& t)
  {
    std::lock_guard<std::mutex> lk(rate_mtx_);
    if (update_min_dt(cam_stamps_[idx], cam_min_dt_per_[idx], t) < 0.0) return;
    double fastest = std::numeric_limits<double>::max();
    for (int i = 0; i < num_cams_; ++i)
      if (static_cast<int>(cam_stamps_[i].size()) >= 2)
        fastest = std::min(fastest, cam_min_dt_per_[i]);
    if (fastest == std::numeric_limits<double>::max()) return;
    cam_min_dt_ = fastest;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "Camera min_dt → %.3f s  bound=%.3f s", cam_min_dt_, cam_min_dt_ * rate_slack_);
    apply_bounds();
  }

  void update_lidar_rate(const rclcpp::Time& t)
  {
    std::lock_guard<std::mutex> lk(rate_mtx_);
    if (update_min_dt(lidar_stamps_, lidar_min_dt_, t) < 0.0) return;
    RCLCPP_INFO_THROTTLE(get_logger(), *get_clock(), 5000,
      "LiDAR min_dt → %.3f s  bound=%.3f s", lidar_min_dt_, lidar_min_dt_ * rate_slack_);
    apply_bounds();
  }

  void apply_bounds()
  {
    if      (sync_cam1_lidar_) set_lidar_bounds(*sync_cam1_lidar_, 1);
    else if (sync2_lidar_)     set_lidar_bounds(*sync2_lidar_,     2);
    else if (sync3_lidar_)     set_lidar_bounds(*sync3_lidar_,     3);
    else if (sync4_lidar_)     set_lidar_bounds(*sync4_lidar_,     4);
    else if (sync5_lidar_)     set_lidar_bounds(*sync5_lidar_,     5);
  }

  template<typename SyncT>
  void set_lidar_bounds(SyncT& sync, int num_cam_inputs)
  {
    const double cam_b   = std::max(cam_min_dt_,   0.001) * rate_slack_;
    const double lidar_b = std::max(lidar_min_dt_, 0.001) * rate_slack_;
    const rclcpp::Duration cam_lb  {0, static_cast<int32_t>(cam_b   * 1e9)};
    const rclcpp::Duration lidar_lb{0, static_cast<int32_t>(lidar_b * 1e9)};
    for (int i = 0; i < num_cam_inputs; ++i) sync.setInterMessageLowerBound(i, cam_lb);
    sync.setInterMessageLowerBound(num_cam_inputs, lidar_lb);
  }

  // ══════════════════════════════════════════════════════
  // Publishers
  // ══════════════════════════════════════════════════════
  void init_pubs()
  {
    image_pubs_.clear(); image_pubs_.reserve(num_cams_);
    for (int i = 0; i < num_cams_; ++i) {
      const std::string out = cameras_[i] + sync_suffix_;
      image_pubs_.push_back(this->create_publisher<ImageMsg>(out, qos_));
      RCLCPP_INFO(get_logger(), "PUB: %s -> %s", cameras_[i].c_str(), out.c_str());
    }
    if (use_lidar_) {
      lidar_pub_ = this->create_publisher<CloudMsg>(lidar_topic_out_, qos_);
      RCLCPP_INFO(get_logger(), "PUB lidar: %s", lidar_topic_out_.c_str());
    }
  }

  // ══════════════════════════════════════════════════════
  // Subscribers + Synchronizers
  // ══════════════════════════════════════════════════════
  void init_subs_and_sync()
  {
    // Build the restamping layer first — this creates raw_cam_subs_,
    // raw_lidar_sub_, cam_pass_, and lidar_pass_.
    setup_restamping_layer();

    if (num_cams_ == 1) {
      if (!use_lidar_) {
        // Single camera, no lidar — raw sub is enough (already created above)
        RCLCPP_INFO(get_logger(), "SUB single (passthrough): %s", cameras_[0].c_str());
        // Override the raw sub to publish directly instead
        raw_cam_subs_[0].reset();
        sub_single_ros_ = this->create_subscription<ImageMsg>(
          cameras_[0], qos_camera_,
          [this](const ImageMsg::ConstSharedPtr msg) {
            auto m = std::make_shared<ImageMsg>(*msg);
            if (restamp_inputs_) m->header.stamp = this->get_clock()->now();
            image_pubs_[0]->publish(*m);
          });
        return;
      }

      // 1 camera + lidar: connect PassThrough outputs to sync
      sync_cam1_lidar_ = std::make_unique<SyncCam1L>(
        PolicyCam1L(queue_size_), *cam_pass_[0], *lidar_pass_);
      apply_bounds();
      sync_cam1_lidar_->registerCallback(
        std::bind(&MultiCameraSync::callback_cam1_lidar, this,
          std::placeholders::_1, std::placeholders::_2));
      RCLCPP_INFO(get_logger(), "SUB: %s", cameras_[0].c_str());
      return;
    }

    // 2-5 cameras: connect PassThrough outputs to sync
    for (int i = 0; i < num_cams_; ++i)
      RCLCPP_INFO(get_logger(), "SUB: %s", cameras_[i].c_str());

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

  void publish_images(const std::vector<ImageMsg::ConstSharedPtr>& imgs)
  { for (size_t i = 0; i < imgs.size(); ++i) image_pubs_[i]->publish(*imgs[i]); }

  void on_sync() { std::lock_guard<std::mutex> lk(diag_mtx_); ++diag_sync_count_; }

  // ══════════════════════════════════════════════════════
  // Callbacks — stamps are already correct here (restamped at input)
  // ══════════════════════════════════════════════════════
  void callback_cam1_lidar(const ImageMsg::ConstSharedPtr& a, const CloudMsg::ConstSharedPtr& cloud)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_lidar_rate(t); image_pubs_[0]->publish(*a); lidar_pub_->publish(*cloud); }

  void callback_cam2(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_cam_rate(1,t); publish_images({a,b}); }

  void callback_cam3(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b, const ImageMsg::ConstSharedPtr& c)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_cam_rate(1,t); update_cam_rate(2,t); publish_images({a,b,c}); }

  void callback_cam4(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b, const ImageMsg::ConstSharedPtr& c, const ImageMsg::ConstSharedPtr& d)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_cam_rate(1,t); update_cam_rate(2,t); update_cam_rate(3,t); publish_images({a,b,c,d}); }

  void callback_cam5(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b, const ImageMsg::ConstSharedPtr& c, const ImageMsg::ConstSharedPtr& d, const ImageMsg::ConstSharedPtr& e)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_cam_rate(1,t); update_cam_rate(2,t); update_cam_rate(3,t); update_cam_rate(4,t); publish_images({a,b,c,d,e}); }

  void callback_cam2_lidar(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b, const CloudMsg::ConstSharedPtr& cloud)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_cam_rate(1,t); update_lidar_rate(t); publish_images({a,b}); lidar_pub_->publish(*cloud); }

  void callback_cam3_lidar(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b, const ImageMsg::ConstSharedPtr& c, const CloudMsg::ConstSharedPtr& cloud)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_cam_rate(1,t); update_cam_rate(2,t); update_lidar_rate(t); publish_images({a,b,c}); lidar_pub_->publish(*cloud); }

  void callback_cam4_lidar(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b, const ImageMsg::ConstSharedPtr& c, const ImageMsg::ConstSharedPtr& d, const CloudMsg::ConstSharedPtr& cloud)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_cam_rate(1,t); update_cam_rate(2,t); update_cam_rate(3,t); update_lidar_rate(t); publish_images({a,b,c,d}); lidar_pub_->publish(*cloud); }

  void callback_cam5_lidar(const ImageMsg::ConstSharedPtr& a, const ImageMsg::ConstSharedPtr& b, const ImageMsg::ConstSharedPtr& c, const ImageMsg::ConstSharedPtr& d, const ImageMsg::ConstSharedPtr& e, const CloudMsg::ConstSharedPtr& cloud)
  { on_sync(); auto t=get_clock()->now(); update_cam_rate(0,t); update_cam_rate(1,t); update_cam_rate(2,t); update_cam_rate(3,t); update_cam_rate(4,t); update_lidar_rate(t); publish_images({a,b,c,d,e}); lidar_pub_->publish(*cloud); }

  // ══════════════════════════════════════════════════════
  // Camera-only synchronizers — use PassThrough as input
  // ══════════════════════════════════════════════════════
  using Policy2 = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg>;
  using Sync2   = message_filters::Synchronizer<Policy2>;
  std::unique_ptr<Sync2> sync2_;
  void init_sync_cam2() {
    sync2_ = std::make_unique<Sync2>(Policy2(queue_size_), *cam_pass_[0], *cam_pass_[1]);
    sync2_->registerCallback(std::bind(&MultiCameraSync::callback_cam2, this, std::placeholders::_1, std::placeholders::_2));
  }

  using Policy3 = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg>;
  using Sync3   = message_filters::Synchronizer<Policy3>;
  std::unique_ptr<Sync3> sync3_;
  void init_sync_cam3() {
    sync3_ = std::make_unique<Sync3>(Policy3(queue_size_), *cam_pass_[0], *cam_pass_[1], *cam_pass_[2]);
    sync3_->registerCallback(std::bind(&MultiCameraSync::callback_cam3, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  using Policy4 = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg>;
  using Sync4   = message_filters::Synchronizer<Policy4>;
  std::unique_ptr<Sync4> sync4_;
  void init_sync_cam4() {
    sync4_ = std::make_unique<Sync4>(Policy4(queue_size_), *cam_pass_[0], *cam_pass_[1], *cam_pass_[2], *cam_pass_[3]);
    sync4_->registerCallback(std::bind(&MultiCameraSync::callback_cam4, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }

  using Policy5 = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg>;
  using Sync5   = message_filters::Synchronizer<Policy5>;
  std::unique_ptr<Sync5> sync5_;
  void init_sync_cam5() {
    sync5_ = std::make_unique<Sync5>(Policy5(queue_size_), *cam_pass_[0], *cam_pass_[1], *cam_pass_[2], *cam_pass_[3], *cam_pass_[4]);
    sync5_->registerCallback(std::bind(&MultiCameraSync::callback_cam5, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
  }

  // ══════════════════════════════════════════════════════
  // Camera + LiDAR synchronizers — use PassThrough as input
  // ══════════════════════════════════════════════════════
  using Policy2L = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, CloudMsg>;
  using Sync2L   = message_filters::Synchronizer<Policy2L>;
  std::unique_ptr<Sync2L> sync2_lidar_;
  void init_sync_cam2_lidar() {
    sync2_lidar_ = std::make_unique<Sync2L>(Policy2L(queue_size_), *cam_pass_[0], *cam_pass_[1], *lidar_pass_);
    apply_bounds();
    sync2_lidar_->registerCallback(std::bind(&MultiCameraSync::callback_cam2_lidar, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3));
  }

  using Policy3L = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, CloudMsg>;
  using Sync3L   = message_filters::Synchronizer<Policy3L>;
  std::unique_ptr<Sync3L> sync3_lidar_;
  void init_sync_cam3_lidar() {
    sync3_lidar_ = std::make_unique<Sync3L>(Policy3L(queue_size_), *cam_pass_[0], *cam_pass_[1], *cam_pass_[2], *lidar_pass_);
    apply_bounds();
    sync3_lidar_->registerCallback(std::bind(&MultiCameraSync::callback_cam3_lidar, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4));
  }

  using Policy4L = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, CloudMsg>;
  using Sync4L   = message_filters::Synchronizer<Policy4L>;
  std::unique_ptr<Sync4L> sync4_lidar_;
  void init_sync_cam4_lidar() {
    sync4_lidar_ = std::make_unique<Sync4L>(Policy4L(queue_size_), *cam_pass_[0], *cam_pass_[1], *cam_pass_[2], *cam_pass_[3], *lidar_pass_);
    apply_bounds();
    sync4_lidar_->registerCallback(std::bind(&MultiCameraSync::callback_cam4_lidar, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5));
  }

  using Policy5L = message_filters::sync_policies::ApproximateTime<ImageMsg, ImageMsg, ImageMsg, ImageMsg, ImageMsg, CloudMsg>;
  using Sync5L   = message_filters::Synchronizer<Policy5L>;
  std::unique_ptr<Sync5L> sync5_lidar_;
  void init_sync_cam5_lidar() {
    sync5_lidar_ = std::make_unique<Sync5L>(Policy5L(queue_size_), *cam_pass_[0], *cam_pass_[1], *cam_pass_[2], *cam_pass_[3], *cam_pass_[4], *lidar_pass_);
    apply_bounds();
    sync5_lidar_->registerCallback(std::bind(&MultiCameraSync::callback_cam5_lidar, this, std::placeholders::_1, std::placeholders::_2, std::placeholders::_3, std::placeholders::_4, std::placeholders::_5, std::placeholders::_6));
  }

  // ══════════════════════════════════════════════════════
  // Member variables
  // ══════════════════════════════════════════════════════
  std::vector<std::string> cameras_;
  std::string              sync_suffix_;
  int                      queue_size_{100};
  int                      num_cams_{0};
  bool                     use_lidar_{false};
  std::string              lidar_topic_in_, lidar_topic_out_;
  bool                     restamp_inputs_{true};

  double                cam_min_dt_{0.060};
  std::array<double,5>  cam_min_dt_per_{0.060,0.060,0.060,0.060,0.060};
  double                lidar_min_dt_{0.085};
  double                rate_slack_{0.85};
  int                   rate_window_{20};

  std::vector<std::deque<rclcpp::Time>> cam_stamps_;
  std::deque<rclcpp::Time>             lidar_stamps_;
  std::mutex                           rate_mtx_;

  rclcpp::QoS       qos_{10};
  rclcpp::QoS       qos_camera_{10};
  rmw_qos_profile_t rmw_qos_{rmw_qos_profile_default};
  rmw_qos_profile_t rmw_qos_camera_{rmw_qos_profile_default};

  std::vector<rclcpp::Publisher<ImageMsg>::SharedPtr> image_pubs_;
  rclcpp::Publisher<CloudMsg>::SharedPtr              lidar_pub_;

  rclcpp::Subscription<CloudMsg>::SharedPtr              diag_lidar_sub_;
  std::vector<rclcpp::Subscription<ImageMsg>::SharedPtr> diag_cam_subs_;
  rclcpp::TimerBase::SharedPtr                           diag_timer_;

  rclcpp::Subscription<ImageMsg>::SharedPtr sub_single_ros_;

  using PolicyCam1L = message_filters::sync_policies::ApproximateTime<ImageMsg, CloudMsg>;
  using SyncCam1L   = message_filters::Synchronizer<PolicyCam1L>;
  std::unique_ptr<SyncCam1L> sync_cam1_lidar_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MultiCameraSync>());
  rclcpp::shutdown();
  return 0;
}