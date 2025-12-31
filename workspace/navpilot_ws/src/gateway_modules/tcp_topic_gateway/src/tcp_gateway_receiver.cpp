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
#include <string>
#include <thread>
#include <vector>

namespace
{
constexpr uint32_t MAGIC = 0x52475731; // 'RGW1'
constexpr uint8_t TOPIC_IMAGE = 1;
constexpr uint8_t TOPIC_INT32 = 2;

struct Header
{
  uint32_t magic_be;
  uint8_t topic_id;
  uint8_t rsv[3];
  uint32_t payload_len_be;
};

static bool recv_all(int fd, uint8_t* data, size_t len)
{
  size_t got = 0;
  while (got < len) {
    ssize_t n = ::recv(fd, data + got, len - got, 0);
    if (n <= 0) return false;
    got += static_cast<size_t>(n);
  }
  return true;
}
} // namespace

class TcpGatewayReceiver : public rclcpp::Node
{
public:
  TcpGatewayReceiver()
  : Node("tcp_gateway_receiver")
  {
    this->declare_parameter<int>("listen_port", 5001);
    this->declare_parameter<std::string>("image_out_topic", "/image_remote");
    this->declare_parameter<std::string>("int_out_topic", "/number_remote");

    listen_port_ = this->get_parameter("listen_port").as_int();
    image_out_topic_ = this->get_parameter("image_out_topic").as_string();
    int_out_topic_ = this->get_parameter("int_out_topic").as_string();

    image_pub_ = this->create_publisher<sensor_msgs::msg::Image>(image_out_topic_, rclcpp::SensorDataQoS());
    int_pub_ = this->create_publisher<std_msgs::msg::Int32>(int_out_topic_, rclcpp::QoS(10));

    running_.store(true);
    net_thread_ = std::thread([this]() { this->server_loop(); });

    RCLCPP_INFO(get_logger(), "Receiver listening on port %d", listen_port_);
  }

  ~TcpGatewayReceiver() override
  {
    running_.store(false);
    if (net_thread_.joinable()) net_thread_.join();
  }

private:
  void server_loop()
  {
    int listen_fd = ::socket(AF_INET, SOCK_STREAM, 0);
    if (listen_fd < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create socket");
      return;
    }

    int yes = 1;
    ::setsockopt(listen_fd, SOL_SOCKET, SO_REUSEADDR, &yes, sizeof(yes));

    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_addr.s_addr = INADDR_ANY;
    addr.sin_port = htons(static_cast<uint16_t>(listen_port_));

    if (::bind(listen_fd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
      RCLCPP_ERROR(get_logger(), "Bind failed");
      ::close(listen_fd);
      return;
    }

    if (::listen(listen_fd, 1) != 0) {
      RCLCPP_ERROR(get_logger(), "Listen failed");
      ::close(listen_fd);
      return;
    }

    while (running_.load()) {
      RCLCPP_INFO(get_logger(), "Waiting for connection...");
      int client_fd = ::accept(listen_fd, nullptr, nullptr);
      if (client_fd < 0) {
        if (!running_.load()) break;
        continue;
      }
      RCLCPP_INFO(get_logger(), "Client connected.");

      // Receive frames until disconnect
      while (running_.load()) {
        Header h{};
        if (!recv_all(client_fd, reinterpret_cast<uint8_t*>(&h), sizeof(h))) {
          RCLCPP_WARN(get_logger(), "Client disconnected (header).");
          break;
        }

        const uint32_t magic = ntohl(h.magic_be);
        const uint32_t payload_len = ntohl(h.payload_len_be);
        const uint8_t topic_id = h.topic_id;

        if (magic != MAGIC) {
          RCLCPP_WARN(get_logger(), "Bad magic. Dropping connection.");
          break;
        }
        if (payload_len == 0 || payload_len > (512u * 1024u * 1024u)) { // 512MB safety
          RCLCPP_WARN(get_logger(), "Bad payload_len=%u. Dropping connection.", payload_len);
          break;
        }

        std::vector<uint8_t> payload(payload_len);
        if (!recv_all(client_fd, payload.data(), payload.size())) {
          RCLCPP_WARN(get_logger(), "Client disconnected (payload).");
          break;
        }

        // Wrap payload into a SerializedMessage for ROS deserialization
        rclcpp::SerializedMessage serialized(payload_len);
        std::memcpy(serialized.get_rcl_serialized_message().buffer, payload.data(), payload_len);
        serialized.get_rcl_serialized_message().buffer_length = payload_len;

        if (topic_id == TOPIC_IMAGE) {
          sensor_msgs::msg::Image msg;
          image_ser_.deserialize_message(&serialized, &msg);
          image_pub_->publish(msg);
        } else if (topic_id == TOPIC_INT32) {
          std_msgs::msg::Int32 msg;
          int_ser_.deserialize_message(&serialized, &msg);
          int_pub_->publish(msg);
        } else {
          // Unknown topic_id, ignore
        }
      }

      ::close(client_fd);
    }

    ::close(listen_fd);
  }

private:
  int listen_port_{5001};
  std::string image_out_topic_;
  std::string int_out_topic_;

  std::atomic<bool> running_{false};
  std::thread net_thread_;

  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
  rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr int_pub_;

  rclcpp::Serialization<sensor_msgs::msg::Image> image_ser_;
  rclcpp::Serialization<std_msgs::msg::Int32> int_ser_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TcpGatewayReceiver>());
  rclcpp::shutdown();
  return 0;
}
