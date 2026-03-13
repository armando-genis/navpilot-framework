#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/bool.hpp"
#include <cstdlib>

class LocalizationSoundAlert : public rclcpp::Node
{
public:
    LocalizationSoundAlert() : Node("localization_sound_alert")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Bool>(
            "/localization_valid",
            10,
            std::bind(&LocalizationSoundAlert::callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Localization sound alert node started");
    }

private:
    void callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        if (!msg->data)
        {
            if (!localization_lost_)
            {
                RCLCPP_WARN(this->get_logger(), "Localization LOST!");

                // Play warning sound
                std::system("paplay /usr/share/sounds/freedesktop/stereo/dialog-warning.oga &");

                localization_lost_ = true;
            }
        }
        else
        {
            localization_lost_ = false;
        }
    }

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr subscription_;
    bool localization_lost_ = false;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalizationSoundAlert>());
    rclcpp::shutdown();
    return 0;
}