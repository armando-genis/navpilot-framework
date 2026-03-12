#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

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
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr uint32_t MAGIC = 0x52475731; // 'RGW1'
constexpr uint8_t TOPIC_IMAGE = 1;
constexpr uint8_t TOPIC_INT32 = 2;

#pragma pack(push, 1)
struct Header
{
  uint32_t magic_be;
  uint8_t topic_id;
  uint8_t camera_id;
  uint8_t rsv[2];
  uint32_t payload_len_be;
};
#pragma pack(pop)

static_assert(sizeof(Header) == 12, "Header must be 12 bytes");

static bool send_all(int fd, const uint8_t* data, size_t len)
{
  size_t sent = 0;
  while (sent < len) {
    ssize_t n = ::send(fd, data + sent, len - sent, MSG_NOSIGNAL);
    if (n <= 0) return false;
    sent += static_cast<size_t>(n);
  }
  return true;
}
} // namespace

class TcpGatewaySender : public rclcpp::Node
{
public:
  TcpGatewaySender()
  : Node("tcp_gateway_sender")
  {
    // Params
    this->declare_parameter<std::string>("remote_ip", "192.168.1.2");
    this->declare_parameter<int>("remote_port", 5001);
    this->declare_parameter<std::string>("int_topic", "/number");
    this->declare_parameter<std::vector<std::string>>(
      "camera_topics",
      {
        "/racecar/camera/camera_0/image_raw",
        "/racecar/camera/camera_1/image_raw",
        "/racecar/camera/camera_2/image_raw"
      });

    remote_ip_ = this->get_parameter("remote_ip").as_string();
    remote_port_ = this->get_parameter("remote_port").as_int();
    camera_topics_ = this->get_parameter("camera_topics").as_string_array();
    int_topic_ = this->get_parameter("int_topic").as_string();

    latest_.resize(camera_topics_.size());

    // Start TCP connection thread
    running_.store(true);
    net_thread_ = std::thread([this]() { this->net_loop(); });

    // Start sender thread (decoupled from ROS callbacks)
    sender_running_.store(true);
    sender_thread_ = std::thread([this]() { this->send_loop(); });

    // QoS: KeepLast(1) + BEST_EFFORT to avoid backlog/latency
    auto img_qos = rclcpp::QoS(rclcpp::KeepLast(1)).best_effort().durability_volatile();

    // Subscriptions: only store latest serialized bytes per camera (overwrite old if unsent)
    for (size_t i = 0; i < camera_topics_.size(); ++i) {
      auto sub = this->create_subscription<sensor_msgs::msg::Image>(
        camera_topics_[i],
        img_qos,
        [this, i](sensor_msgs::msg::Image::ConstSharedPtr msg)
        {
          // Serialize quickly and store bytes (no send here)
          rclcpp::SerializedMessage serialized;
          image_ser_.serialize_message(msg.get(), &serialized);

          const size_t n = serialized.size();
          std::vector<uint8_t> out(n);
          std::memcpy(out.data(),
                      serialized.get_rcl_serialized_message().buffer,
                      n);

          {
            std::lock_guard<std::mutex> lk(frame_mtx_);
            if (latest_[i].ready) {
              dropped_.fetch_add(1, std::memory_order_relaxed); // overwrite old frame
            }
            latest_[i].bytes = std::move(out);
            latest_[i].ready = true;
          }
          frame_cv_.notify_one();
        });

      image_subs_.push_back(sub);

      RCLCPP_INFO(get_logger(), "Subscribed camera %zu: %s",
                  i, camera_topics_[i].c_str());
    }

    // Int topic (optional) — can remain reliable since it's tiny
    int_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      int_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        this->send_int_msg(*msg);
      });

    RCLCPP_INFO(get_logger(), "Sender ready. Connecting to %s:%d",
                remote_ip_.c_str(), remote_port_);
  }

  ~TcpGatewaySender() override
  {
    running_.store(false);
    sender_running_.store(false);
    frame_cv_.notify_all();

    if (net_thread_.joinable()) net_thread_.join();
    if (sender_thread_.joinable()) sender_thread_.join();

    close_socket();
  }

private:
  struct PendingFrame {
    std::vector<uint8_t> bytes;
    bool ready{false};
  };

  void close_socket()
  {
    std::lock_guard<std::mutex> lk(sock_mtx_);
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
    }
  }

  bool is_connected()
  {
    std::lock_guard<std::mutex> lk(sock_mtx_);
    return sock_fd_ >= 0;
  }

  int get_sock_fd()
  {
    std::lock_guard<std::mutex> lk(sock_mtx_);
    return sock_fd_;
  }

  void net_loop()
  {
    while (running_.load()) {
      if (is_connected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        continue;
      }

      int fd = ::socket(AF_INET, SOCK_STREAM, 0);
      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      sockaddr_in addr{};
      addr.sin_family = AF_INET;
      addr.sin_port = htons(static_cast<uint16_t>(remote_port_));
      if (::inet_pton(AF_INET, remote_ip_.c_str(), &addr.sin_addr) != 1) {
        ::close(fd);
        RCLCPP_ERROR(get_logger(), "Invalid remote_ip: %s", remote_ip_.c_str());
        std::this_thread::sleep_for(std::chrono::seconds(2));
        continue;
      }

      RCLCPP_INFO(get_logger(), "Connecting to %s:%d ...",
                  remote_ip_.c_str(), remote_port_);

      if (::connect(fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        ::close(fd);
        RCLCPP_WARN(get_logger(), "Connect failed. Retrying...");
        std::this_thread::sleep_for(std::chrono::seconds(1));
        continue;
      }

      // Reduce latency: disable Nagle
      int flag = 1;
      ::setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &flag, sizeof(flag));

      {
        std::lock_guard<std::mutex> lk(sock_mtx_);
        sock_fd_ = fd;
      }
      RCLCPP_INFO(get_logger(), "Connected!");
    }
  }

  void send_int_msg(const std_msgs::msg::Int32 & msg)
  {
    int fd = get_sock_fd();
    if (fd < 0) return;

    rclcpp::SerializedMessage serialized;
    int_ser_.serialize_message(&msg, &serialized);

    const size_t payload_len = serialized.size();
    Header h{};
    h.magic_be = htonl(MAGIC);
    h.topic_id = TOPIC_INT32;
    h.camera_id = 0;
    h.rsv[0] = h.rsv[1] = 0;
    h.payload_len_be = htonl(static_cast<uint32_t>(payload_len));

    std::lock_guard<std::mutex> lk(send_mtx_);
    if (!send_all(fd, reinterpret_cast<const uint8_t*>(&h), sizeof(h)) ||
        !send_all(fd,
                  reinterpret_cast<const uint8_t*>(serialized.get_rcl_serialized_message().buffer),
                  payload_len))
    {
      RCLCPP_WARN(get_logger(), "INT send failed, closing socket.");
      close_socket();
    }
  }

  void send_loop()
  {
    auto last_log = std::chrono::steady_clock::now();

    while (sender_running_.load()) {
      // Wait until there is at least one ready frame or timeout
      {
        std::unique_lock<std::mutex> lk(frame_mtx_);
        frame_cv_.wait_for(lk, std::chrono::milliseconds(20), [this](){
          if (!sender_running_.load()) return true;
          for (auto &f : latest_) if (f.ready) return true;
          return false;
        });
      }
      if (!sender_running_.load()) break;

      int fd = get_sock_fd();
      if (fd < 0) {
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
        continue;
      }

      // Send newest frame per camera (if available)
      for (size_t cam = 0; cam < latest_.size(); ++cam) {
        std::vector<uint8_t> payload;
        {
          std::lock_guard<std::mutex> lk(frame_mtx_);
          if (!latest_[cam].ready) continue;
          payload = std::move(latest_[cam].bytes);
          latest_[cam].ready = false;
        }

        Header h{};
        h.magic_be = htonl(MAGIC);
        h.topic_id = TOPIC_IMAGE;
        h.camera_id = static_cast<uint8_t>(cam);
        h.rsv[0] = h.rsv[1] = 0;
        h.payload_len_be = htonl(static_cast<uint32_t>(payload.size()));

        // Send header+payload atomically
        std::lock_guard<std::mutex> lk(send_mtx_);
        if (!send_all(fd, reinterpret_cast<const uint8_t*>(&h), sizeof(h)) ||
            !send_all(fd, payload.data(), payload.size()))
        {
          RCLCPP_WARN(get_logger(), "Image send failed, closing socket.");
          close_socket();
          break;
        }
      }

      // Log drops once per second
      auto now = std::chrono::steady_clock::now();
      if (now - last_log > std::chrono::seconds(1)) {
        const uint64_t d = dropped_.exchange(0, std::memory_order_relaxed);
        if (d > 0) {
          RCLCPP_WARN(get_logger(), "Dropped %lu frames (overwritten unsent)", (unsigned long)d);
        }
        last_log = now;
      }
    }
  }

private:
  std::string remote_ip_;
  int remote_port_;
  std::vector<std::string> camera_topics_;
  std::string int_topic_;

  std::atomic<bool> running_{false};
  std::thread net_thread_;

  std::atomic<bool> sender_running_{false};
  std::thread sender_thread_;

  std::mutex sock_mtx_;
  std::mutex send_mtx_;
  int sock_fd_{-1};

  std::mutex frame_mtx_;
  std::condition_variable frame_cv_;
  std::vector<PendingFrame> latest_;
  std::atomic<uint64_t> dropped_{0};

  std::vector<rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr> image_subs_;
  rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr int_sub_;

  rclcpp::Serialization<sensor_msgs::msg::Image> image_ser_;
  rclcpp::Serialization<std_msgs::msg::Int32> int_ser_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpGatewaySender>());
  rclcpp::shutdown();
  return 0;
}