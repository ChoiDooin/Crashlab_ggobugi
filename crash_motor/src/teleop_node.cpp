#include "rclcpp/rclcpp.hpp"  // rclcpp 관련 헤더 포함
#include "std_msgs/msg/int64_multi_array.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <chrono>
#include <signal.h>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <memory>  // 스마트 포인터 사용을 위한 헤더

#define KEYCODE_RIGHT 0x43
#define KEYCODE_LEFT 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_SPACE 0x20
#define KEYCODE_Q 0x71
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77

using namespace std::chrono_literals;

class KeyboardReader {
public:
  KeyboardReader() : kfd(0) {
    tcgetattr(kfd, &cooked_);
    struct termios raw;
    memcpy(&raw, &cooked_, sizeof(struct termios));
    raw.c_lflag &= ~(ICANON | ECHO);
    raw.c_cc[VEOL] = 1;
    raw.c_cc[VEOF] = 2;
    tcsetattr(kfd, TCSANOW, &raw);
  }

  ~KeyboardReader() {
    tcsetattr(kfd, TCSANOW, &cooked_);
  }

  void readOne(char *c) {
    int rc = read(kfd, c, 1);
    if (rc < 0) {
      throw std::runtime_error("Failed to read keyboard input");
    }
  }

private:
  int kfd;
  struct termios cooked_;
};

class RosCommunicator : public rclcpp::Node {
public:
  RosCommunicator()
      : Node("tutorial_teleop"), dir_1_(0), dir_2_(0), pwm_1_(0), pwm_2_(0), desired_pwm_(30),
        keyboard_reader_() {
    teleop_publisher_ = this->create_publisher<std_msgs::msg::Int64MultiArray>("/tutorial/teleop", 10);
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    timer_ = this->create_wall_timer(10ms, std::bind(&RosCommunicator::TimerCallback, this));

    puts("Reading from keyboard...");
    puts("---------------------------");
    puts("Use arrow keys to move the robot:");
    puts("↑ : Forward    ↓ : Backward");
    puts("← : Left       → : Right");
    puts("Space: Stop");
  }

private:
  void TimerCallback() {
    char c;
    try {
      keyboard_reader_.readOne(&c);
    } catch (const std::runtime_error &e) {
      RCLCPP_ERROR(this->get_logger(), "Keyboard read error: %s", e.what());
      return;
    }

    // Process keyboard input
    switch (c) {
      case KEYCODE_LEFT:
        dir_1_ = 1;
        dir_2_ = 1;
        pwm_1_ = desired_pwm_;
        pwm_2_ = desired_pwm_;
        break;
      case KEYCODE_RIGHT:
        dir_1_ = 0;
        dir_2_ = 0;
        pwm_1_ = desired_pwm_;
        pwm_2_ = desired_pwm_;
        break;
      case KEYCODE_UP:
        dir_1_ = 0;
        dir_2_ = 1;
        pwm_1_ = desired_pwm_;
        pwm_2_ = desired_pwm_;
        break;
      case KEYCODE_DOWN:
        dir_1_ = 1;
        dir_2_ = 0;
        pwm_1_ = desired_pwm_;
        pwm_2_ = desired_pwm_;
        break;
      case KEYCODE_SPACE:
        dir_1_ = 2;
        dir_2_ = 2;
        pwm_1_ = 0;
        pwm_2_ = 0;
        break;
      case KEYCODE_Q:
        RCLCPP_INFO(this->get_logger(), "Quit signal received");
        rclcpp::shutdown();
        return;
      default:
        break;
      case KEYCODE_W:
        if (pwm_1_ > 0 && pwm_2_ > 0){
          pwm_1_ += 5;
          pwm_2_ += 5;
        }
        break;
      case KEYCODE_S:
      if (pwm_1_ > 0 && pwm_2_ > 0){
          pwm_1_ -= 5;
          pwm_2_ -= 5;
        }
    }

    // Publish teleop message
    auto teleop_message = std_msgs::msg::Int64MultiArray();
    teleop_message.data = {dir_1_, dir_2_, pwm_1_, pwm_2_};
    teleop_publisher_->publish(teleop_message);

    // Publish cmd_vel message
    auto twist_message = geometry_msgs::msg::Twist();
    twist_message.linear.x = (dir_1_ == 0 && dir_2_ == 1) ? 0.3 : (dir_1_ == 1 && dir_2_ == 0) ? -0.3 : (dir_1_ == 2 && dir_2_ == 2) ? 0.0 : 0.0;
    twist_message.angular.z = (dir_1_ == 1 && dir_2_ == 1) ? 0.5 : (dir_1_ == 0 && dir_2_ == 0) ? -0.5 : (dir_1_ == 2 && dir_2_ == 2) ? 0.0 : 0.0;
    cmd_vel_publisher_->publish(twist_message);
  }

  rclcpp::Publisher<std_msgs::msg::Int64MultiArray>::SharedPtr teleop_publisher_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
  rclcpp::TimerBase::SharedPtr timer_;

  int dir_1_;
  int dir_2_;
  int pwm_1_;
  int pwm_2_;
  int desired_pwm_;
  KeyboardReader keyboard_reader_;
};

void quitHandler(int sig) {
  (void)sig;
  rclcpp::shutdown();
}

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  signal(SIGINT, quitHandler);
  rclcpp::spin(std::make_shared<RosCommunicator>());

  rclcpp::shutdown();
  return 0;
}

