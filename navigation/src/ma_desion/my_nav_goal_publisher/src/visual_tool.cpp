#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>

class CmdVelProcessor : public rclcpp::Node {
public:
    CmdVelProcessor() : Node("cmd_vel_processor") {
        subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "/cmd_vel", 10, std::bind(&CmdVelProcessor::cmdVelCallback, this, std::placeholders::_1));
        publisher_ = this->create_publisher<std_msgs::msg::Float32>("/runtime", 10);
    }

private:
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        // 计算线速度的大小
        float linear_speed = std::sqrt(
            std::pow(msg->linear.x, 2) +
            std::pow(msg->linear.y, 2) +
            std::pow(msg->linear.z, 2)
        );

        // 发布到 /runtime
        auto message = std_msgs::msg::Float32();
        message.data = linear_speed;
        publisher_->publish(message);

        RCLCPP_INFO(this->get_logger(), "Published linear speed: %.2f", message.data);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr publisher_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CmdVelProcessor>());
    rclcpp::shutdown();
    return 0;
}
