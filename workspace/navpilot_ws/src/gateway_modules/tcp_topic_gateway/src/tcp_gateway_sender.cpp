#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/int32.hpp>

#include <rclcpp/serialization.hpp>
#include <rclcpp/serialized_message.hpp>

#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <atomic>
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

// Fixed 12-byte header:
// [0..3]  magic (uint32, network order)
// [4]     topic_id (uint8)
// [5..7]  reserved (uint8[3]=0)
// [8..11] payload_len (uint32, network order)
struct Header
{
  uint32_t magic_be;
  uint8_t topic_id;
  uint8_t rsv[3];
  uint32_t payload_len_be;
};

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
    this->declare_parameter<std::string>("remote_ip", "192.168.1.2");
    this->declare_parameter<int>("remote_port", 5001);
    this->declare_parameter<std::string>("image_topic", "/image");
    this->declare_parameter<std::string>("int_topic", "/number");

    remote_ip_ = this->get_parameter("remote_ip").as_string();
    remote_port_ = this->get_parameter("remote_port").as_int();
    image_topic_ = this->get_parameter("image_topic").as_string();
    int_topic_ = this->get_parameter("int_topic").as_string();

    // Start TCP connection thread
    running_.store(true);
    net_thread_ = std::thread([this]() { this->net_loop(); });

    // Subscriptions
    image_sub_ = this->create_subscription<sensor_msgs::msg::Image>(
      image_topic_, rclcpp::SensorDataQoS(),
      [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
        this->send_msg(TOPIC_IMAGE, *msg, image_ser_);
      });

    int_sub_ = this->create_subscription<std_msgs::msg::Int32>(
      int_topic_, rclcpp::QoS(10),
      [this](std_msgs::msg::Int32::ConstSharedPtr msg) {
        this->send_msg(TOPIC_INT32, *msg, int_ser_);
      });

    RCLCPP_INFO(get_logger(), "Sender ready. Will connect to %s:%d",
                remote_ip_.c_str(), remote_port_);
  }

  ~TcpGatewaySender() override
  {
    running_.store(false);
    if (net_thread_.joinable()) net_thread_.join();
    close_socket();
  }

private:
  void close_socket()
  {
    std::lock_guard<std::mutex> lk(sock_mtx_);
    if (sock_fd_ >= 0) {
      ::close(sock_fd_);
      sock_fd_ = -1;
    }
  }

  void net_loop()
  {
    while (running_.load()) {
      // If connected, just sleep a bit; send_msg() uses the socket directly.
      if (is_connected()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(250));
        continue;
      }

      // Try connect
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

      {
        std::lock_guard<std::mutex> lk(sock_mtx_);
        sock_fd_ = fd;
      }
      RCLCPP_INFO(get_logger(), "Connected!");
    }
  }

  bool is_connected()
  {
    std::lock_guard<std::mutex> lk(sock_mtx_);
    return sock_fd_ >= 0;
  }

  template <typename MsgT>
  void send_msg(uint8_t topic_id, const MsgT & msg, rclcpp::Serialization<MsgT> & ser)
  {
    int fd;
    {
      std::lock_guard<std::mutex> lk(sock_mtx_);
      fd = sock_fd_;
    }
    if (fd < 0) return; // not connected

    rclcpp::SerializedMessage serialized;
    ser.serialize_message(&msg, &serialized);

    const size_t payload_len = serialized.size();
    Header h{};
    h.magic_be = htonl(MAGIC);
    h.topic_id = topic_id;
    h.rsv[0] = h.rsv[1] = h.rsv[2] = 0;
    h.payload_len_be = htonl(static_cast<uint32_t>(payload_len));

    // Send header + payload atomically under a send mutex
    std::lock_guard<std::mutex> lk(send_mtx_);
    if (!send_all(fd, reinterpret_cast<const uint8_t*>(&h), sizeof(h)) ||
        !send_all(fd, reinterpret_cast<const uint8_t*>(serialized.get_rcl_serialized_message().buffer),
                  payload_len))
    {
      RCLCPP_WARN(get_logger(), "Send failed, closing socket.");
      close_socket();
    }
  }

private:
  std::string remote_ip_;
  int remote_port_;
  std::string image_topic_;
  std::string int_topic_;

  std::atomic<bool> running_{false};
  std::thread net_thread_;

  std::mutex sock_mtx_;
  std::mutex send_mtx_;
  int sock_fd_{-1};

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
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
