#include <crash_motor/motor_node.hpp>
#include "rclcpp/rclcpp.hpp"

class TestRunner : public rclcpp::Node
{
    public:
        subscriber_ = this->create_subscription<nav_msgs::msg::Odom>(
            "/odom", 10, std::bind(&TestRunner::OdomCallback, this, std::placeholders::_1));
};