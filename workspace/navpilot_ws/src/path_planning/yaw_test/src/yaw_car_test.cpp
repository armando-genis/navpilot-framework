#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float64.hpp>

class YawCarTestPublisher : public rclcpp::Node
{
public:
    YawCarTestPublisher() : Node("yaw_car_test_publisher"), current_yaw_(-0.3)
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64>("/sdv/steering/position", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&YawCarTestPublisher::timerCallback, this));
    }

private:
    void timerCallback()
    {
        auto message = std_msgs::msg::Float64();
        message.data = current_yaw_;
        publisher_->publish(message);
        // RCLCPP_INFO(this->get_logger(), "Publishing: '%f'", message.data);
        current_yaw_ += yaw_increment_;
        if (current_yaw_ > 0.6 || current_yaw_ < -0.6)
        {
            yaw_increment_ *= -1;  // Change direction
        }
    }

    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    double current_yaw_;
    double yaw_increment_ = 0.02;  // Faster increment
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<YawCarTestPublisher>());
    rclcpp::shutdown();
    return 0;
}
