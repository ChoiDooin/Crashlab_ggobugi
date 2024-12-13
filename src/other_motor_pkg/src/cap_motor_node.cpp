#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/string.hpp"
#include <pigpiod_if2.h>

// 모터 제어 핀 설정
#define DIR_PIN 20  // 방향 핀
#define STEP_PIN 21 // 스텝 핀

int pi;
class CapMotorNode : public rclcpp::Node
{
public:
    CapMotorNode() : Node("cap_motor_node")
    {

        // Publisher

        // Subscriber
        go_sub_ = this->create_subscription<std_msgs::msg::Float64>("/go", 10, std::bind(&CapMotorNode::GoCallback, this, std::placeholders::_1));

        // Timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&CapMotorNode::TimerCallback, this));
    }

private:
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr go_sub_;

    rclcpp::TimerBase::SharedPtr timer_;

    // 변수 선언
    bool goNow = false;

    void GoCallback(const std_msgs::msg::Float64::SharedPtr)
    {
        goNow = true;
    }
    void TimerCallback()
    {
        initMotor();
        if (goNow)
        {
            int steps = 100; // 1.8도 스텝 각도를 가진 모터의 경우 180도 회전을 위한 스텝 수
            for (int i = 0; i < steps; i++)
            {
                gpio_write(pi, STEP_PIN, 1); // 스텝 핀을 HIGH로 설정
            }
        }
        else
        {
            int steps = 100;
            for (int i = 0; i < steps; i++)
            {
                gpio_write(pi, STEP_PIN, 0); // 스텝 핀을 HIGH로 설정
            }
        }
        RCLCPP_INFO(rclcpp::get_logger(""), "HI");
    }

    void initMotor()
    {
        pi = pigpio_start(NULL, NULL); // pigpio 데몬에 연결
        if (pi < 0)
        {
            fprintf(stderr, "pigpio 데몬에 연결 실패.\n");
            return;
        }

        // 핀 모드 설정
        set_mode(pi, DIR_PIN, PI_OUTPUT);
        set_mode(pi, STEP_PIN, PI_OUTPUT);

        // 초기 핀 상태 설정
        gpio_write(pi, DIR_PIN, 0);  // 방향 핀 LOW
        gpio_write(pi, STEP_PIN, 0); // 스텝 핀 LOW
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<CapMotorNode>());
    rclcpp::shutdown();
    return 0;
}