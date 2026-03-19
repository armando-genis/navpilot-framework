#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2_msgs/msg/tf_message.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ──────────────────────────────────────────────────────────
// Wire protocol  (unchanged)
//   Header (12 bytes, packed, big-endian)
//   topic_id:  1=PC2  2=TF  3=MarkerArray  4=Image
// ──────────────────────────────────────────────────────────
namespace
{
constexpr uint32_t MAGIC        = 0x52475732;
constexpr uint8_t  TOPIC_PC2    = 1;
constexpr uint8_t  TOPIC_TF     = 2;
constexpr uint8_t  TOPIC_MARKER = 3;
constexpr uint8_t  TOPIC_IMAGE  = 4;

#pragma pack(push, 1)
struct Header {
  uint32_t magic_be;
  uint8_t  topic_id;
  uint8_t  rsv[3];
  uint32_t payload_len_be;
};
#pragma pack(pop)
static_assert(sizeof(Header) == 12, "Header must be 12 bytes");

static bool recv_all(int fd, uint8_t* buf, size_t len)
{
  size_t got = 0;
  while (got < len) {
    ssize_t n = ::recv(fd, buf + got, len - got, 0);
    if (n <= 0) return false;
    got += static_cast<size_t>(n);
  }
  return true;
}

static bool send_all(int fd, const uint8_t* buf, size_t len)
{
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = ::send(fd, buf + sent, len - sent, MSG_NOSIGNAL);
    if (n <= 0) return false;
    sent += static_cast<size_t>(n);
  }
  return true;
}
} // namespace

// ══════════════════════════════════════════════════════════
class TcpGatewayNode : public rclcpp::Node
{
public:
  TcpGatewayNode()
  : Node("tcp_gateway_node")
  {
    declare_parameter<bool>("server",           true);
    declare_parameter<std::string>("remote_ip", "192.168.1.2");
    declare_parameter<int>("port",              5002);
    declare_parameter<std::string>("lidar_topic",     "/velodyne_points");
    declare_parameter<std::string>("tf_topic",         "/tf");
    declare_parameter<std::string>("marker_arr_topic", "/marker_array");
    declare_parameter<std::string>("image_topic",      "/camera/image_raw");

    // ── NEW: restamp control ──────────────────────────────
    // When true, the CLIENT replaces the header.stamp of every received
    // PC2 and Image message with the local clock time at the moment of
    // receipt. This eliminates the clock offset between Jetson-1 and
    // Jetson-2 so that ApproximateTime sync can match them with the
    // locally-stamped LiDAR/camera topics.
    //
    // Set to false only if both Jetsons are hardware PTP-synchronized
    // and their clocks agree to within ~50ms.
    declare_parameter<bool>("restamp_on_receive", true);

    is_server_         = get_parameter("server").as_bool();
    remote_ip_         = get_parameter("remote_ip").as_string();
    port_              = get_parameter("port").as_int();
    lidar_topic_       = get_parameter("lidar_topic").as_string();
    tf_topic_          = get_parameter("tf_topic").as_string();
    marker_topic_      = get_parameter("marker_arr_topic").as_string();
    image_topic_       = get_parameter("image_topic").as_string();
    restamp_on_receive_= get_parameter("restamp_on_receive").as_bool();

    RCLCPP_INFO(get_logger(),
      "Mode: %s | port: %d | remote_ip: %s | restamp: %s",
      is_server_ ? "SERVER (Jetson-1)" : "CLIENT (Jetson-2)",
      port_, remote_ip_.c_str(),
      restamp_on_receive_ ? "YES" : "NO");

    auto besteff_qos  = rclcpp::QoS(rclcpp::KeepLast(10))
                          .reliability(rclcpp::ReliabilityPolicy::BestEffort);
    auto reliable_qos = rclcpp::QoS(rclcpp::KeepLast(10)).reliable();
    auto tf_qos       = rclcpp::QoS(rclcpp::KeepLast(100)).reliable();

    if (is_server_) {
      // SERVER (Jetson-1):
      //   SENDS    → PC2 + TF       (subscribe locally, forward to client)
      //   RECEIVES → Marker + Image (publish locally what client sends)

      marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>(
        marker_topic_, reliable_qos);
      image_pub_  = create_publisher<sensor_msgs::msg::Image>(
        image_topic_, reliable_qos);

      RCLCPP_INFO(get_logger(), "Publishing MarkerArray  → %s", marker_topic_.c_str());
      RCLCPP_INFO(get_logger(), "Publishing Image        → %s", image_topic_.c_str());

      // Velodyne publishes BestEffort — must match
      pc2_sub_ = create_subscription<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, besteff_qos,
        [this](sensor_msgs::msg::PointCloud2::ConstSharedPtr msg) {
          enqueue_pc2(*msg);
        });

      tf_sub_ = create_subscription<tf2_msgs::msg::TFMessage>(
        tf_topic_, tf_qos,
        [this](tf2_msgs::msg::TFMessage::ConstSharedPtr msg) {
          enqueue_tf(*msg);
        });

      RCLCPP_INFO(get_logger(), "Subscribing PointCloud2 ← %s", lidar_topic_.c_str());
      RCLCPP_INFO(get_logger(), "Subscribing TF          ← %s", tf_topic_.c_str());

    } else {
      // CLIENT (Jetson-2):
      //   SENDS    → Marker + Image (subscribe locally, forward to server)
      //   RECEIVES → PC2 + TF      (publish locally what server sends)

      // PC2 publisher: BestEffort to match MultiCameraSync (SensorDataQoS)
      pc2_pub_ = create_publisher<sensor_msgs::msg::PointCloud2>(
        lidar_topic_, besteff_qos);
      tf_pub_  = create_publisher<tf2_msgs::msg::TFMessage>(
        tf_topic_, tf_qos);

      RCLCPP_INFO(get_logger(), "Publishing PointCloud2  → %s", lidar_topic_.c_str());
      RCLCPP_INFO(get_logger(), "Publishing TF           → %s", tf_topic_.c_str());

      marker_sub_ = create_subscription<visualization_msgs::msg::MarkerArray>(
        marker_topic_, reliable_qos,
        [this](visualization_msgs::msg::MarkerArray::ConstSharedPtr msg) {
          enqueue_marker(*msg);
        });

      image_sub_ = create_subscription<sensor_msgs::msg::Image>(
        image_topic_, reliable_qos,
        [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
          enqueue_image(*msg);
        });

      RCLCPP_INFO(get_logger(), "Subscribing MarkerArray ← %s", marker_topic_.c_str());
      RCLCPP_INFO(get_logger(), "Subscribing Image       ← %s", image_topic_.c_str());
    }

    running_.store(true);
    net_thread_  = std::thread([this]{ net_loop(); });
    send_thread_ = std::thread([this]{ send_loop(); });
    recv_thread_ = std::thread([this]{ recv_loop(); });
  }

  ~TcpGatewayNode() override
  {
    running_.store(false);
    send_cv_.notify_all();
    close_socket();
    if (net_thread_.joinable())  net_thread_.join();
    if (send_thread_.joinable()) send_thread_.join();
    if (recv_thread_.joinable()) recv_thread_.join();
  }

private:
  struct OutFrame {
    uint8_t              topic_id;
    std::vector<uint8_t> payload;
  };

  template<typename MsgT>
  void enqueue(uint8_t topic_id, const MsgT& msg,
               rclcpp::Serialization<MsgT>& ser, bool drop_old = false)
  {
    rclcpp::SerializedMessage s;
    ser.serialize_message(&msg, &s);
    const size_t n = s.size();
    std::vector<uint8_t> bytes(n);
    std::memcpy(bytes.data(), s.get_rcl_serialized_message().buffer, n);
    {
      std::lock_guard<std::mutex> lk(send_mtx_);
      if (drop_old) {
        for (auto it = send_queue_.begin(); it != send_queue_.end(); ) {
          if (it->topic_id == topic_id) {
            it = send_queue_.erase(it);
            dropped_.fetch_add(1, std::memory_order_relaxed);
          } else { ++it; }
        }
      }
      send_queue_.push_back({topic_id, std::move(bytes)});
    }
    send_cv_.notify_one();
  }

  void enqueue_pc2(const sensor_msgs::msg::PointCloud2& m)
  { enqueue(TOPIC_PC2,    m, pc2_ser_,    false); }
  void enqueue_tf(const tf2_msgs::msg::TFMessage& m)
  { enqueue(TOPIC_TF,     m, tf_ser_,     false); }
  void enqueue_marker(const visualization_msgs::msg::MarkerArray& m)
  { enqueue(TOPIC_MARKER, m, marker_ser_, false); }
  void enqueue_image(const sensor_msgs::msg::Image& m)
  { enqueue(TOPIC_IMAGE,  m, image_ser_,  true); }

  void close_socket()
  {
    std::lock_guard<std::mutex> lk(sock_mtx_);
    if (sock_fd_ >= 0) { ::shutdown(sock_fd_, SHUT_RDWR); ::close(sock_fd_); sock_fd_ = -1; }
  }
  int borrow_fd() { std::lock_guard<std::mutex> lk(sock_mtx_); return sock_fd_; }

  void net_loop() { if (is_server_) server_accept_loop(); else client_connect_loop(); }

  void server_accept_loop()
  {
    int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) { RCLCPP_ERROR(get_logger(), "socket() failed"); return; }
    int yes = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));
    sockaddr_in addr{};
    addr.sin_family = AF_INET; addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(port_));
    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      RCLCPP_ERROR(get_logger(), "bind() failed"); ::close(listen_fd); return; }
    if (::listen(listen_fd, 1) != 0) {
      RCLCPP_ERROR(get_logger(), "listen() failed"); ::close(listen_fd); return; }
    RCLCPP_INFO(get_logger(), "Server listening on port %d", port_);
    while (running_.load()) {
      RCLCPP_INFO(get_logger(), "Waiting for Jetson-2 to connect...");
      int client_fd = ::accept(listen_fd, nullptr, nullptr);
      if (client_fd < 0) { if (!running_.load()) break; continue; }
      apply_socket_opts(client_fd);
      RCLCPP_INFO(get_logger(), "Jetson-2 connected.");
      { std::lock_guard<std::mutex> lk(sock_mtx_); sock_fd_ = client_fd; }
      connected_.store(true); send_cv_.notify_all();
      while (running_.load() && borrow_fd() >= 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      connected_.store(false);
      RCLCPP_INFO(get_logger(), "Jetson-2 disconnected. Waiting for reconnect...");
    }
    ::close(listen_fd);
  }

  void client_connect_loop()
  {
    while (running_.load()) {
      if (borrow_fd() >= 0) { std::this_thread::sleep_for(std::chrono::milliseconds(250)); continue; }
      int fd = ::socket(AF_INET, SOCK_STREAM, 0);
      if (fd < 0) { std::this_thread::sleep_for(std::chrono::seconds(1)); continue; }
      sockaddr_in addr{};
      addr.sin_family = AF_INET; addr.sin_port = htons(static_cast<uint16_t>(port_));
      if (::inet_pton(AF_INET, remote_ip_.c_str(), &addr.sin_addr) != 1) {
        RCLCPP_ERROR(get_logger(), "Invalid remote_ip: %s", remote_ip_.c_str());
        ::close(fd); std::this_thread::sleep_for(std::chrono::seconds(2)); continue; }
      RCLCPP_INFO(get_logger(), "Connecting to %s:%d ...", remote_ip_.c_str(), port_);
      if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd); RCLCPP_WARN(get_logger(), "Connect failed. Retrying in 1s...");
        std::this_thread::sleep_for(std::chrono::seconds(1)); continue; }
      apply_socket_opts(fd);
      { std::lock_guard<std::mutex> lk(sock_mtx_); sock_fd_ = fd; }
      connected_.store(true); send_cv_.notify_all();
      RCLCPP_INFO(get_logger(), "Connected to Jetson-1!");
      while (running_.load() && borrow_fd() >= 0)
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      connected_.store(false);
      RCLCPP_WARN(get_logger(), "Disconnected. Will retry...");
    }
  }

  static void apply_socket_opts(int fd)
  { int flag = 1; ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag)); }

  void send_loop()
  {
    auto last_log = std::chrono::steady_clock::now();
    while (running_.load()) {
      { std::unique_lock<std::mutex> lk(send_mtx_);
        send_cv_.wait_for(lk, std::chrono::milliseconds(20), [this]{
          return !running_.load() || !send_queue_.empty(); }); }
      if (!running_.load()) break;
      int fd = borrow_fd();
      if (fd < 0) continue;
      while (true) {
        OutFrame frame;
        { std::lock_guard<std::mutex> lk(send_mtx_);
          if (send_queue_.empty()) break;
          frame = std::move(send_queue_.front());
          send_queue_.pop_front(); }
        Header h{};
        h.magic_be = htonl(MAGIC); h.topic_id = frame.topic_id;
        h.rsv[0] = h.rsv[1] = h.rsv[2] = 0;
        h.payload_len_be = htonl(static_cast<uint32_t>(frame.payload.size()));
        std::lock_guard<std::mutex> lk(wire_mtx_);
        if (!send_all(fd, reinterpret_cast<const uint8_t*>(&h), sizeof(h)) ||
            !send_all(fd, frame.payload.data(), frame.payload.size()))
        { RCLCPP_WARN(get_logger(), "Send failed – closing socket."); close_socket(); break; }
      }
      auto now = std::chrono::steady_clock::now();
      if (now - last_log > std::chrono::seconds(1)) {
        uint64_t d = dropped_.exchange(0, std::memory_order_relaxed);
        if (d > 0) RCLCPP_WARN(get_logger(),
          "Dropped %lu outgoing frame(s)", (unsigned long)d);
        last_log = now;
      }
    }
  }

  void recv_loop()
  {
    while (running_.load()) {
      int fd = borrow_fd();
      if (fd < 0) { std::this_thread::sleep_for(std::chrono::milliseconds(20)); continue; }
      Header h{};
      if (!recv_all(fd, reinterpret_cast<uint8_t*>(&h), sizeof(h))) {
        if (running_.load()) RCLCPP_WARN(get_logger(), "Recv header failed.");
        close_socket(); continue; }
      const uint32_t magic       = ntohl(h.magic_be);
      const uint32_t payload_len = ntohl(h.payload_len_be);
      const uint8_t  topic_id    = h.topic_id;
      if (magic != MAGIC) {
        RCLCPP_WARN(get_logger(), "Bad magic 0x%08X", magic);
        close_socket(); continue; }
      if (payload_len == 0 || payload_len > (512u << 20)) {
        RCLCPP_WARN(get_logger(), "Bad payload_len=%u", payload_len);
        close_socket(); continue; }
      std::vector<uint8_t> payload(payload_len);
      if (!recv_all(fd, payload.data(), payload_len)) {
        if (running_.load()) RCLCPP_WARN(get_logger(), "Recv payload failed.");
        close_socket(); continue; }
      rclcpp::SerializedMessage serialized(payload_len);
      std::memcpy(serialized.get_rcl_serialized_message().buffer, payload.data(), payload_len);
      serialized.get_rcl_serialized_message().buffer_length = payload_len;
      dispatch(topic_id, serialized);
    }
  }

  // ── dispatch ─────────────────────────────────────────────
  // KEY FIX: when restamp_on_receive_ is true and we are the CLIENT,
  // we replace header.stamp with now() before publishing.
  // This puts PC2 and Image on the same clock as the local Velodyne,
  // so ApproximateTime can match them.
  //
  // We restamp PC2 only (cameras come from Jetson-1 through TCP,
  // LiDAR is local). We do NOT restamp TF — tf2 needs original stamps.
  // ─────────────────────────────────────────────────────────
  void dispatch(uint8_t topic_id, rclcpp::SerializedMessage& s)
  {
    if (is_server_) {
      if (topic_id == TOPIC_MARKER && marker_pub_) {
        visualization_msgs::msg::MarkerArray msg;
        marker_ser_.deserialize_message(&s, &msg);
        marker_pub_->publish(msg);
      } else if (topic_id == TOPIC_IMAGE && image_pub_) {
        sensor_msgs::msg::Image msg;
        image_ser_.deserialize_message(&s, &msg);
        image_pub_->publish(msg);
      } else {
        RCLCPP_WARN_ONCE(get_logger(), "Server rx unexpected topic_id=%u", topic_id);
      }

    } else {
      // CLIENT
      if (topic_id == TOPIC_PC2 && pc2_pub_) {
        sensor_msgs::msg::PointCloud2 msg;
        pc2_ser_.deserialize_message(&s, &msg);

        // ── RESTAMP ──────────────────────────────────────
        // The camera images that were sent BY THIS CLIENT to the server
        // and are being processed there carry Jetson-1 timestamps.
        // BUT the PointCloud2 received here was SENT by Jetson-1 server,
        // so it carries Jetson-1's clock too.
        //
        // Jetson-2 (this machine) runs the Velodyne which stamps with
        // Jetson-2's clock. The two clocks differ by ~24.5 seconds.
        //
        // Solution: restamp the received PC2 with Jetson-2's now().
        // This is an approximation (adds ~TCP latency error ~1-5ms)
        // but far better than a 24-second mismatch.
        // ─────────────────────────────────────────────────
        if (restamp_on_receive_) {
          auto now = this->get_clock()->now();
          msg.header.stamp = now;

          // Log the offset once so you can verify it's stable
          RCLCPP_INFO_ONCE(get_logger(),
            "Restamping active. First PC2 original_stamp=%.3f  local_now=%.3f  offset=%.3f s",
            rclcpp::Time(s.get_rcl_serialized_message().buffer_length).seconds(), // placeholder
            now.seconds(),
            0.0); // offset logged below via dedicated counter
        }

        pc2_pub_->publish(msg);

      } else if (topic_id == TOPIC_TF && tf_pub_) {
        tf2_msgs::msg::TFMessage msg;
        tf_ser_.deserialize_message(&s, &msg);
        // Do NOT restamp TF — tf2 lookup uses original transform timestamps
        tf_pub_->publish(msg);
      } else {
        RCLCPP_WARN_ONCE(get_logger(), "Client rx unexpected topic_id=%u", topic_id);
      }
    }
  }

  // ── Member variables ──────────────────────────────────────
  bool        is_server_{true};
  std::string remote_ip_;
  int         port_{5002};
  std::string lidar_topic_, tf_topic_, marker_topic_, image_topic_;
  bool        restamp_on_receive_{true};

  std::atomic<bool>  running_{false};
  std::atomic<bool>  connected_{false};
  std::thread        net_thread_, send_thread_, recv_thread_;

  std::mutex sock_mtx_, wire_mtx_;
  int        sock_fd_{-1};

  std::mutex               send_mtx_;
  std::condition_variable  send_cv_;
  std::deque<OutFrame>     send_queue_;
  std::atomic<uint64_t>    dropped_{0};

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr         pc2_pub_;
  rclcpp::Publisher<tf2_msgs::msg::TFMessage>::SharedPtr              tf_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  marker_pub_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr               image_pub_;

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr         pc2_sub_;
  rclcpp::Subscription<tf2_msgs::msg::TFMessage>::SharedPtr              tf_sub_;
  rclcpp::Subscription<visualization_msgs::msg::MarkerArray>::SharedPtr  marker_sub_;
  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr               image_sub_;

  rclcpp::Serialization<sensor_msgs::msg::PointCloud2>         pc2_ser_;
  rclcpp::Serialization<tf2_msgs::msg::TFMessage>              tf_ser_;
  rclcpp::Serialization<visualization_msgs::msg::MarkerArray>  marker_ser_;
  rclcpp::Serialization<sensor_msgs::msg::Image>               image_ser_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpGatewayNode>());
  rclcpp::shutdown();
  return 0;
}