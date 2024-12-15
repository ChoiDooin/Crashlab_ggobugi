#include <nav_motor_pkg/motor_node.hpp>
#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include "tf2/convert.h"
#include "tf2_ros/buffer.h"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_listener.h"

class TestRunner : public rclcpp::Node
{
public:
    TestRunner() : Node("test_running_node"), tf_buffer_(this->get_clock()), tf_listener_(std::make_shared<tf2_ros::TransformListener>(tf_buffer_))
    {
        goal_pwm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/goal_pwm", 10);

        cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>("/cmd_vel", 10, std::bind(&TestRunner::CmdVelCallback, this, std::placeholders::_1));
        goal_location_sub_ = this->create_subscription<std_msgs::msg::String>("/hospital_location", 10, std::bind(&TestRunner::GoalLocationCallback, this, std::placeholders::_1));
        cam_data_sub_ = this->create_subscription<geometry_msgs::msg::Point>("/json_data", 10, std::bind(&TestRunner::CamDataCallback, this, std::placeholders::_1));
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odom", 10, std::bind(&TestRunner::OdomCallback, this, std::placeholders::_1));

        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&TestRunner::TimerCallback, this));
    }

private:
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr goal_pwm_pub_;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr goal_location_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr cam_data_sub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    tf2_ros::Buffer tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

    rclcpp::TimerBase::SharedPtr timer_;

    double slam_linear_velocity = 0.0, slam_angular_velocity = 0.0;
    double linear_velocity = 0.0, angular_velocity = 0.0;
    double left_pwm = 0.0, right_pwm = 0.0;

    double cam_pose_x = 0.0, cam_pose_y = 0.0;
    double cam_area = 0.0;

    double now_pos_x = 0.0, now_pos_y = 0.0, now_pos_heading = 0.0;

    bool goLocation = true;

    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
        slam_linear_velocity = msg->linear.x;
        slam_angular_velocity = msg->angular.z;
    }

    void GoalLocationCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        goLocation = true;
    }

    void CamDataCallback(const geometry_msgs::msg::Point::SharedPtr msg)
    {
        cam_pose_x = msg->x;
        cam_pose_y = msg->y;
        cam_area = msg->z;
    }

    void OdomCallback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        now_pos_x = msg->pose.pose.position.x;
        now_pos_y = msg->pose.pose.position.y;

        // 메시지에서 쿼터니언을 추출
        tf2::Quaternion q(
        msg->pose.pose.orientation.x,
        msg->pose.pose.orientation.y,
        msg->pose.pose.orientation.z,
        msg->pose.pose.orientation.w);

        // 쿼터니언을 롤, 피치, 요로 변환
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        roll = roll * (180 / M_PI);
        pitch = pitch * (180 / M_PI);
        now_pos_heading = yaw * (180 / M_PI);
        
        RCLCPP_INFO(rclcpp::get_logger("Odometry"), "x: %f  ||   y: %f   ||   roll: %f   ||   pitch: %f   ||   now_heading : %f", now_pos_x, now_pos_y, roll, pitch, now_pos_heading);
     }

    void TimerCallback()
    {
        if (goLocation)
        {
            // if(now_pos_heading > -0.2){
            //     left_pwm = -45;
            //     right_pwm = -45;
            // }
            // else {
            //     left_pwm = 0;
            //     right_pwm = 0;
            // }
            // RCLCPP_INFO(this->get_logger(Callback), "TimerCallback");
            
            // 유저 인식 넓이에 따른 속도 조절

            // if (cam_area >200000){
            //     linear_velocity = slam_linear_velocity;
            //     angular_velocity = slam_angular_velocity;
            // }
            // else if (cam_area > 130000){
            //     linear_velocity = slam_linear_velocity * 0.8;
            //     angular_velocity = slam_angular_velocity * 0.8;
            // }
            // else if (cam_area > 80000){
            //     linear_velocity = slam_linear_velocity * 0.5;
            //     angular_velocity = slam_angular_velocity * 0.5;
            // }
            // else {
            //     linear_velocity = 0;
            //     angular_velocity = 0;
            // }
            linear_velocity = slam_linear_velocity * 2.0;
            angular_velocity = slam_angular_velocity;
            // linear_velocity = slam_linear_velocity;
            // angular_velocity = slam_angular_velocity;

            // RCLCPP_INFO(rclcpp::get_logger("cmd vel"),"linear : %f   ||   angular : %f",linear_velocity, angular_velocity);
            calculateWheelPWM(linear_velocity, angular_velocity, left_pwm, right_pwm);
            PublishGoalPwm(left_pwm, right_pwm);
        }
        else
        {
            if (cam_area > 180000) // 사람이 화면 가까이 위치
            {
                if (cam_pose_x > 350) // 사람이 화면 상 오른쪽에 위치
                {
                    left_pwm = 35; // 왼쪽으로 회전
                    right_pwm = 35;
                }
                else if (cam_pose_x > 290) // 사람이 화면 상 중간에 위치
                {
                    left_pwm = 0; // 정지
                    right_pwm = 0;
                }
                else // 사람이 화면 상 왼쪽에 위치
                {
                    left_pwm = -35; // 오른쪽으로 회전
                    right_pwm = -35;
                }
            }
            else if(cam_area > 10000)
            {                  // 사람이 화면 멀리에 위치
                left_pwm = 50; // 전진
                right_pwm = -50;
            }
            else {
                left_pwm = 0;
                right_pwm = 0;
            }
            PublishGoalPwm(left_pwm, right_pwm);
        }

        
        // RCLCPP_INFO(rclcpp::get_logger("Goal PWM"), "left : %f     ||     right: %f\n", left_pwm, right_pwm);
    }

    void PublishGoalPwm(double target_pwm1, double target_pwm2)
    {
        auto goal_pwm_msg = std::make_shared<std_msgs::msg::Float64MultiArray>();
        goal_pwm_msg->data = {target_pwm1, target_pwm2};
        goal_pwm_pub_->publish(*goal_pwm_msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TestRunner>());
    rclcpp::shutdown();
    return 0;
}