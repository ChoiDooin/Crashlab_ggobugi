#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include <chrono>
#include <memory>
#include <iostream> // 터미널 입력을 위한 헤더 파일 추가

class PIDGainPublisher : public rclcpp::Node
{
public:
    PIDGainPublisher()
    : Node("pid_gain_publisher")
    {
        publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("pid_gain", 10);
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(1000),
            std::bind(&PIDGainPublisher::publish_pid_gain, this));
    }

private:
    void publish_pid_gain()
    {
        std::vector<double> gains(6, 0.0); // PID 게인을 저장할 벡터 초기화

        std::cout << "Enter PID gains (P1, I1, D1, P2, I2, D2): ";
        for (double& gain : gains) {
            std::cin >> gain; // 터미널에서 각 게인 입력받기
        }

        auto message = std_msgs::msg::Float64MultiArray();
        message.data = gains; // 입력받은 게인으로 메시지 데이터 설정
        RCLCPP_INFO(this->get_logger(), "Publishing: '%f', '%f', '%f', '%f', '%f', '%f'", message.data[0], message.data[1], message.data[2], message.data[3], message.data[4], message.data[5]);
        publisher_->publish(message);
    }

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<PIDGainPublisher>());

  rclcpp::shutdown();
  return 0;
}

