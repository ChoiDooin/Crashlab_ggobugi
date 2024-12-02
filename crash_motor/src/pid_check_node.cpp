#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

class RPMSubscriber : public rclcpp::Node
{
public:
    RPMSubscriber() : Node("rpm_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "/rpm", 10, std::bind(&RPMSubscriber::topic_callback, this, std::placeholders::_1));

        publisher1_ = this->create_publisher<std_msgs::msg::Float64>("pid_check_rpm1", 10);
        publisher2_ = this->create_publisher<std_msgs::msg::Float64>("pid_check_rpm2", 10);
        publisher3_ = this->create_publisher<std_msgs::msg::Float64>("pid_check_target_rpm1", 10);
        publisher4_ = this->create_publisher<std_msgs::msg::Float64>("pid_check_target_rpm2", 10);
    }

private:
    void topic_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != 4)
        {
            RCLCPP_ERROR(this->get_logger(), "Received data size is not correct for rpm");
            return;
        }

        double rpm_value1 = msg->data[0];
        double rpm_value2 = msg->data[1];
        double target_rpm1 = msg->data[2];
        double target_rpm2 = msg->data[3];

        RCLCPP_INFO(this->get_logger(),
                    "Received rpm values: rpm_value1: '%f', rpm_value2: '%f', target_rpm1: '%f', target_rpm2: '%f'",
                    rpm_value1, rpm_value2, target_rpm1, target_rpm2);

        publish_value(publisher1_, rpm_value1);
        publish_value(publisher2_, rpm_value2);
        publish_value(publisher3_, -target_rpm1);
        publish_value(publisher4_, -target_rpm2);

        printf("RPMs\n");
        printf("rpm_value1: %f\n", rpm_value1);
        printf("rpm_value2: %f\n", rpm_value2);
        printf("target_rpm1: %f\n", target_rpm1);
        printf("target_rpm2: %f\n", target_rpm2);
    }

    void publish_value(const rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr &publisher, double value)
    {
        auto pub_msg = std_msgs::msg::Float64();
        pub_msg.data = value;
        publisher->publish(pub_msg);
    }

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher1_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher2_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher3_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr publisher4_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RPMSubscriber>());
    rclcpp::shutdown();
    return 0;
}
