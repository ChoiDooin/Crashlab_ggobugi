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
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_msgs/msg/string.hpp"
#include "tf2/utils.h"

// 모터 관련 전역 변수 선언
extern double rpm_value1;
extern double rpm_value2;
extern double current_pwm1;
extern double current_pwm2;
extern bool current_direction1;
extern bool current_direction2;
extern int speed_count_1;
extern int speed_count_2;
extern int encoder_count_1A;
extern int encoder_count_1B;
extern int encoder_count_2A;
extern int encoder_count_2B;
extern int encoder_count_1;
extern int encoder_count_2;
extern int encoder_resolution;

// 오도메트리 관련 전역 변수 선언
extern double x;
extern double y;
extern double heading;
extern double left_wheel_old_pos;
extern double right_wheel_old_pos;
extern double left_wheel_new_pos;

// 시스템 관련 전역 변수 선언
extern double control_cycle;
extern double acceleration_ratio;
extern double wheel_radius;
extern double robot_radius;
extern double wheel_round;
extern double robot_round;
extern bool switch_direction;
extern int theta_distance_flag;

// 모터 핀 번호
extern int pinum;
extern int motor1_dir;
extern int motor2_dir;
extern int motor1_pwm;
extern int motor2_pwm;
extern int motor1_encA;
extern int motor1_encB;
extern int motor2_encA;
extern int motor2_encB;

// PWM 관련 전역 변수 선언
extern int pwm_frequency;
extern int pwm_range;
extern int pwm_limit;

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

// PWM 제한 함수
int LimitPwm(int pwm);

// RPM 계산 함수
void CalculateRpm(double dt, double &now_rpm1, double &now_rpm2);

// 오도메트리 계산 함수
void CalculateOdom(double dt, double &delta_linear, double &delta_angular);

void PidController(double p_gain1, double i_gain1, double d_gain1, double errorGap1, double &prevError1, double &sumError1, double target_rpm1, double current_rpm1, double time_interval, double &filteredPidControl1,
                double p_gain2, double i_gain2, double d_gain2, double errorGap2, double &prevError2, double &sumError2, double target_rpm2, double current_rpm2, double &filteredPidControl2, double max_spike_threshold);
// 모터 상태 출력 함수
void InfoMotors();

void calculateWheelPWM(double linear_velocity, double angular_velocity, double &pwm_left, double &pwm_right);
#endif // MOTOR_NODE_HPP
