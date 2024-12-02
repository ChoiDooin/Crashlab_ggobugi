#include <crash_motor/motor_node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/int64_multi_array.hpp"
#include "std_msgs/msg/int64.hpp"
#include "std_msgs/msg/float64.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include <chrono>

class MotorDriver : public rclcpp::Node
{
public:
  MotorDriver()
      : Node("motor_test_node"),
        state_(),
        tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
  // clock_(RCL_ROS_TIME) //ROS Time
  {
    // this->declare_parameter("use_sim_time", rclcpp::ParameterValue(true));
    // this->set_parameter(rclcpp::Parameter("use_sim_time", true));
    // Publisher
    odom_publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
    rpm_publisher_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/rpm", 10);

    // Subscriber
    teleop_subscriber_ = this->create_subscription<std_msgs::msg::Int64MultiArray>(
        "/tutorial/teleop", 10, std::bind(&MotorDriver::TeleopCallback, this, std::placeholders::_1));
    pid_gain_subscriber_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
        "/pid_gain", 10, std::bind(&MotorDriver::PidGainCallback, this, std::placeholders::_1));
    cmd_vel_subscriber_ = this->create_subscription<geometry_msgs::msg::Twist>(
        "/cmd_vel", 10, std::bind(&MotorDriver::cmdVelCallback, this, std::placeholders::_1));

    // Timer
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&MotorDriver::run, this));

    // 초기화
    Initialize();

    // LoadParameters(state_);
    // InitMotors(state_);
    // InitEncoders(state_);
    // SetInterrupts(state_);

    last_update_time_ = this->now();
  }

private:
  SystemState state_;

  // ROS2 요소
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_publisher_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr rpm_publisher_;
  rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_gain_subscriber_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscriber_;
  rclcpp::Subscription<std_msgs::msg::Int64MultiArray>::SharedPtr teleop_subscriber_;
  rclcpp::TimerBase::SharedPtr timer_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  rclcpp::Time last_update_time_;

  // PID 파라미터
  double p_gain1_ = 6.88, d_gain1_ = 0.01;
  double p_gain2_ = 6.3, d_gain2_ = 0.01;
  double errorGap1_ = 0.0, errorGap2_ = 0.0;
  double prevError1_ = 0.0, prevError2_ = 0.0;

  double target_pwm1 = 0.0, target_pwm2 = 0.0;
  double left_wheel_rpm = 0.0, right_wheel_rpm = 0.0;
  bool direction1 = true, direction2 = true;
  bool direction_flag1 = true, direction_flag2 = true;
  double current_pwm1 = 0.0, current_pwm2 = 0.0;

  double now_rpm1 = 0.0, now_rpm2 = 0.0;

  double linear_velocity = 0.0, angular_velocity = 0.0;
  double left_wheel_velocity = 0.0, right_wheel_velocity = 0.0;
  double left_wheel_pwm = 0.0, right_wheel_pwm = 0.0;
  double wheel_base = 20.52;
  double wheel_radius_teleop = 5.75;
  double to_see_target1 = 0.0, to_see_target2 = 0.0;
  bool accelated = true;
  double abs_target_pwm1 = 0.0, abs_target_pwm2 = 0.0;

  void TeleopCallback(const std_msgs::msg::Int64MultiArray::SharedPtr msg)
  {
    // accelated = false;
    //  if (msg->data[0] == 0)
    //    direction1 = true;
    //  else
    //    direction1 = false;

    // if (msg->data[1] == 0)
    //   direction2 = true;
    // else
    //   direction2 = false;

    // int desired_pwm1 = msg->data[2];
    // int desired_pwm2 = msg->data[3];

    // left_wheel_pwm = (direction1 ? -1 : 1 ) * desired_pwm1;
    // right_wheel_pwm = (direction2 ? -1 : 1 ) * desired_pwm2;

    // AccelController(1, direction1, desired_pwm1);
    // AccelController(2, direction2, desired_pwm2);
    // accelated = true;
  }

  void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    linear_velocity = msg->linear.x;
    angular_velocity = msg->angular.z;
    left_wheel_velocity = linear_velocity - (angular_velocity * wheel_base / 2);
    right_wheel_velocity = linear_velocity + (angular_velocity * wheel_base / 2);
    if (left_wheel_velocity > 5){
      left_wheel_velocity = 1;
    }
    if (right_wheel_velocity > 5){
      right_wheel_velocity = 1;
    }
    left_wheel_rpm = (left_wheel_velocity / (2 * M_PI * wheel_radius_teleop)) * 60;   // target_rpm1 //왼쪽 = CCW가 전진
    right_wheel_rpm = (right_wheel_velocity / (2 * M_PI * wheel_radius_teleop)) * 60; // target_rpm2
    left_wheel_pwm = -(left_wheel_rpm / 100) * 250;
    right_wheel_pwm = (right_wheel_rpm / 100) * 250;
    // target_pwm1 = left_wheel_pwm;
    // target_pwm2 = right_wheel_pwm;

    // to_see_target1 = left_wheel_rpm;
    // to_see_target2 = right_wheel_rpm;

    if (left_wheel_pwm > 0)
      direction1 = true;
    if (left_wheel_pwm < 0)
      direction1 = false;

    if (right_wheel_pwm > 0)
      direction2 = true;
    if (right_wheel_pwm < 0)
      direction2 = false;
  }
  void PidGainCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
  {
    p_gain1_ = msg->data[0];
    d_gain1_ = msg->data[1];
    p_gain2_ = msg->data[2];
    d_gain2_ = msg->data[3];
  }

  void run()
  {
    // oneTimeEncoder();
    //  RCLCPP_INFO(this->get_logger(),"Before CalculateRpm: spee_count_1 = %d, speed_count_2 = %d"
    //  ,state_.motor_state.speed_count_1,state_.motor_state.speed_count_2);
    rclcpp::Time current_time = this->now();
    if (current_time.get_clock_type() != last_update_time_.get_clock_type())
    {
      RCLCPP_ERROR(this->get_logger(), "Time sources are different");

      return;
    }

    // double dt = std::max((current_time - last_update_time_).seconds(), 0.001);
    // last_update_time_ = current_time;

    double dt = 0.01; 

    CalculateRpm(dt, now_rpm1, now_rpm2);

    // state_.motor_state.speed_count_1 = 0;
    // state_.motor_state.speed_count_2 = 0;

    // RCLCPP_INFO(this->get_logger(),"After CalculateRpm: rpm_value_1 = %10.0f, rpm_value_2 = %10.0f"
    // ,state_.motor_state.rpm_value1,state_.motor_state.rpm_value2);

    // 모터 정보 출력
    //InfoMotors();
    printf("Target Direction1 : %d    ||  Target Direction2 : %d\n", direction1, direction2);
    printf("Target RPM1 : %10.0f    ||  Target RPM2 : %10.0f\n", left_wheel_rpm, right_wheel_rpm);
    // Delta time 계산

    // 모터 상태 업데이트 (RPM 계산)
    CalculateRpm(dt, now_rpm1, now_rpm2);
    // state_.motor_state.speed_count_1 = 0;
    // state_.motor_state.speed_count_2 = 0;
    current_pwm1 = (now_rpm1 / 100) * 250; // Max RPM = 4000, PWM limit = 250, Reduction ratio = 12
    current_pwm2 = (now_rpm2 / 100) * 250;

    double pidControl1, pidControl2;

    // CalculateError(left_wheel_rpm, state_.motor_state.rpm_value1, errorGap1_);
    // PidController(p_gain1_, d_gain1_, errorGap1_, prevError1_,left_wheel_rpm, now_rpm1, dt, pidControl1);

    PidController(p_gain1_, d_gain1_, errorGap1_, prevError1_, left_wheel_pwm, current_pwm1, dt, pidControl1,
                  p_gain2_, d_gain2_, errorGap2_, prevError2_, right_wheel_pwm, current_pwm2, pidControl2);
    // target_pwm1 = (pidControl1 /150) * 333.33;
    // target_pwm1 = (pidControl1 / 90) *150;
    // CalculateError(right_wheel_rpm, state_.motor_state.rpm_value2, errorGap2_);

    // PidController(p_gain2_, d_gain2_, errorGap2_, prevError2_, right_wheel_pwm, current_pwm2, dt, pidControl2);
    // target_pwm2 = (pidControl2 / 150) * 333.33;
    // target_pwm2 = (pidControl2 / 90) *150;

    printf("Target PWM1: %10.f      ||  Target PWM2 : %10.0f", left_wheel_pwm, right_wheel_pwm);

    RCLCPP_INFO(rclcpp::get_logger("Running"), "pdControl1: %10f   ||   pdControl2: %10f", pidControl1, pidControl2);
    // 방향 및 PWM 설정
    target_pwm1 = abs(pidControl1);
    target_pwm2 = abs(pidControl2);

    if (direction1)
    {
      direction_flag1 = true;
    }
    // else if (target_pwm1 == 0)
    //   target_pwm1 = 0;
    else
    {
      direction_flag1 = false;
    }

    if (direction2)
    {
      direction_flag2 = true;
      
    }
    // else if (target_pwm2 == 0)
    //   target_pwm2 = 0;
    else
    {
      direction_flag2 = false;
    }

    // // PWM limit
    // target_pwm1 = std::clamp(target_pwm1, 0.0, 150.0);
    // target_pwm2 = std::clamp(target_pwm2, 0.0, 150.0);
    RCLCPP_INFO(rclcpp::get_logger("Running"), "pidPWM1: %10f   ||   pidPWM2: %10f", target_pwm1, target_pwm2);
    // PID 결과로 모터 제어 (예제의 PID 기반 제어를 유지)
    MotorController(1, direction_flag1, static_cast<int>(target_pwm1));
    MotorController(2, direction_flag2, static_cast<int>(target_pwm2));

    // 오도메트리 계산
    double delta_linear, delta_angular;
    CalculateOdom(dt, delta_linear, delta_angular);

    // 오도메트리와 TF 브로드캐스트
    PublishOdom(delta_linear, delta_angular);
    // PID 확인용 토픽 퍼블리싱
    std_msgs::msg::Float64MultiArray msg;
    msg.data = {
        current_pwm1,
        current_pwm2,
        left_wheel_pwm, // 목표 RPM (왼쪽)
        right_wheel_pwm // 목표 RPM (오른쪽)
    };
    rpm_publisher_->publish(msg);
  }

  void PublishOdom(double delta_linear, double delta_angular)
  {
    auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
    odom_msg->header.stamp = this->now();
    odom_msg->header.frame_id = "odom";
    odom_msg->child_frame_id = "base_footprint";

    // 오도메트리 데이터 채우기
    odom_msg->pose.pose.position.x = state_.odom_state.x;
    odom_msg->pose.pose.position.y = state_.odom_state.y;
    odom_msg->pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state_.odom_state.heading);
    odom_msg->pose.pose.orientation = tf2::toMsg(q);

    odom_msg->twist.twist.linear.x = delta_linear;
    odom_msg->twist.twist.angular.z = delta_angular;

    odom_publisher_->publish(*odom_msg);

    // Transform Broadcaster
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = this->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_footprint";
    transform.transform.translation.x = state_.odom_state.x;
    transform.transform.translation.y = state_.odom_state.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_msg->pose.pose.orientation;

    tf_broadcaster_->sendTransform(transform);
  }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MotorDriver>());
  rclcpp::shutdown();

  // MotorController(state_.motor_state, 1, true, 0, state_);
  // MotorController(state_.motor_state, 2, true, 0, state_);

  return 0;
}
