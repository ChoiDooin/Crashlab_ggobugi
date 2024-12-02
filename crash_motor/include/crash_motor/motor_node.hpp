#ifndef MOTOR_NODE_HPP
#define MOTOR_NODE_HPP

#include <cmath>
#include <string>
#include <fstream>
#include <iostream>
#include <pigpiod_if2.h>
#include <deque>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include "ament_index_cpp/get_package_share_directory.hpp"


// 모터 상태 구조체
struct MotorState {
    double rpm_value1 = 0.0;
    double rpm_value2 = 0.0;
    double current_pwm1 = 0.0;
    double current_pwm2 = 0.0;
    bool current_direction1 = true;
    bool current_direction2 = true;
    int speed_count_1 = 0;
    int speed_count_2 = 0;
    int encoder_count_1A = 0;
    int encoder_count_1B = 0;
    int encoder_count_2A = 0;
    int encoder_count_2B = 0;
    int encoder_count_1 = 0;
    int encoder_count_2 = 0;
};

// 오도메트리 상태 구조체
struct OdomState {
    double x = 0.0;
    double y = 0.0;
    double heading = 0.0;
    double left_wheel_old_pos = 0.0;
    double right_wheel_old_pos = 0.0;
};

// 시스템 상태 구조체
struct SystemState {
    MotorState motor_state;
    OdomState odom_state;

    int pinum = -1;
    int motor1_dir = 19;
    int motor2_dir = 6;
    int motor1_pwm = 26;
    int motor2_pwm = 13;
    int motor1_encA = 23;
    int motor1_encB = 24;
    int motor2_encA = 27;
    int motor2_encB = 17;

    int pwm_frequency = 40000;
    int pwm_range = 512;
    int pwm_limit = 100;
    int encoder_resolution = 228;
    double control_cycle = 10;
    double acceleration_ratio = 25;
    double wheel_radius = 5.75;
    double robot_radius = 20.52;
    double wheel_round = 2 * M_PI * 5.75;
    double robot_round = 2 * M_PI * 20.52;
    bool switch_direction = true;
    int theta_distance_flag = 0;
};

// 파라미터 로드 함수
void LoadParameters();

// 모터 초기화 함수
int InitMotors();

// 인터럽트 설정 함수
void SetInterrupts();

// 엔코더 초기화 함수
void InitEncoders();

// 엔코더 합산 함수
int SumMotor1Encoder();
int SumMotor2Encoder();

void Initialize();

// 모터 제어 함수
void MotorController(int motor_num, bool direction, int pwm);

// 가속 제어 함수
void AccelController(int motor_num, bool direction, int desired_pwm);

// PWM 제한 함수
int LimitPwm(int pwm);

//RPM 계산 함수
void CalculateRpm(double dt, double &now_rpm1, double &now_rpm2);

// 오도메트리 계산 함수
void CalculateOdom(double dt, double &delta_linear, double &delta_angular);

// PID 제어 관련 함수

void PidController(double p_gain1, double d_gain1, double errorGap1, double &prevError1, double target_rpm1, double current_rpm1, double time_interval, double &pidControl1,double p_gain2, double d_gain2, double errorGap2, double &prevError2, double target_rpm2, double current_rpm2, double &pidControl2);
// 로봇 동작 함수
void ThetaTurn(double theta, int pwm, SystemState &system_state);
void DistanceGo(double distance, int pwm, SystemState &system_state);
void ThetaTurnDistanceGo(double theta, int turn_pwm, double distance, int go_pwm, SystemState &system_state);

// 모터 상태 출력 함수
void InfoMotors();

//Test
void oneTimeEncoder();
#endif // MOTOR_NODE_HPP
