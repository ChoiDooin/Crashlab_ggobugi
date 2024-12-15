#include <nav_motor_pkg/motor_node.hpp>

class MotorRunNode : public rclcpp::Node
{
public:
    MotorRunNode() : Node("motor_run_node"), tf_broadcaster_(std::make_shared<tf2_ros::TransformBroadcaster>(this))
    {
        // Publisher
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom", 10);
        pwm_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>("/pwm", 10);

        // Subscriber
        goal_pwm_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/goal_pwm", 10, std::bind(&MotorRunNode::GoalPwmCallback, this, std::placeholders::_1));
        pid_gain_sub_ = this->create_subscription<std_msgs::msg::Float64MultiArray>("/pid_gain", 10, std::bind(&MotorRunNode::PidGainCallback, this, std::placeholders::_1));

        // Timer
        timer_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&MotorRunNode::TimerCallback, this));

        Initialize(); // 초기화
    }

private:
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr pwm_pub_;

    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr goal_pwm_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pid_gain_sub_;

    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

    // 변수 선언

    double goal_pwm1 = 0.0, goal_pwm2 = 0.0;
    double now_rpm1 = 0.0, now_rpm2 = 0.0;

    // PID 변수
    double pid_control1 = 0.0, pid_control2 = 0.0;
    double error_gap1 = 0.0, prev_error1 = 0.0, sumError1 = 0.0;
    double error_gap2 = 0.0, prev_error2 = 0.0, sumError2 = 0.0;
    double pid_to_pwm1 = 0.0, pid_to_pwm2 = 0.0;

    double p_gain1 = 0.0;
    double i_gain1 = 0.0;
    double d_gain1 = 0.0;
    double p_gain2 = 0.0;
    double i_gain2 = 0.0;
    double d_gain2 = 0.0;
    double dt = 0.01;
    double control_cycle = 10.0;

    // PWM 변수
    double now_pwm1 = 0.0, now_pwm2 = 0.0;
    double target_pwm1 = 0.0, target_pwm2 = 0.0;

    // 방향 변수
    bool direction1 = true, direction2 = true;
    bool direction_flag1 = true, direction_flag2 = true;

    // Odom 변수
    double delta_linear = 0.0, delta_angular = 0.0;

    void GoalPwmCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        goal_pwm1 = msg->data[0];
        goal_pwm2 = msg->data[1];

        if (goal_pwm1 > 0)
            direction1 = true;
        if (goal_pwm1 < 0)
            direction1 = false;

        if (goal_pwm2 > 0)
            direction2 = true;
        if (goal_pwm2 < 0)
            direction2 = false;
    }

    void PidGainCallback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        p_gain1 = msg->data[0];
        i_gain1 = msg->data[1];
        d_gain1 = msg->data[2];
        p_gain2 = msg->data[3];
        i_gain2 = msg->data[4];
        d_gain2 = msg->data[5];
    }
    
    void TimerCallback()
    {
        // RPM 계산
        CalculateRpm(control_cycle, now_rpm1, now_rpm2);

        // RPM을 PWM으로 변환
        // now_pwm1 = (now_rpm1 / ((3 / wheel_radius) * (60 / (2 * M_PI)))) * 100;
        // now_pwm2 = (now_rpm2 / ((3 / wheel_radius) * (60 / (2 * M_PI)))) * 100;
        now_pwm1 = (now_rpm1 / 100) * 150;
        now_pwm2 = (now_rpm2 / 100) * 150;
        if (goal_pwm1 > 0 && goal_pwm2 > 0)
        {
            PidController(0.55, 0.0, 0.0015, error_gap1, prev_error1, sumError1, goal_pwm1, now_pwm1, dt, pid_control1,
                          0.45, 0.0, 0.0015, error_gap2, prev_error2, sumError2, goal_pwm2, now_pwm2, pid_control2, 15);
        }
        else if (goal_pwm1 < 0 && goal_pwm2 < 0)
        {
            PidController(0.52, 0.0, 0.0015, error_gap1, prev_error1, sumError1, goal_pwm1, now_pwm1, dt, pid_control1,
                          0.54, 0.0, 0.0015, error_gap2, prev_error2, sumError2, goal_pwm2, now_pwm2, pid_control2, 15);
        }

        else
        {
            // PID 제어
            PidController(0.45, 0.0, 0.0015, error_gap1, prev_error1, sumError1, goal_pwm1, now_pwm1, dt, pid_control1,
                          0.43, 0.0, 0.0015, error_gap2, prev_error2, sumError2, goal_pwm2, now_pwm2, pid_control2, 30);
        }

        // // // PID 제어
        // PidController(p_gain1, i_gain1, d_gain1, error_gap1, prev_error1, sumError1, goal_pwm1, now_pwm1, dt, pid_control1,
        //               p_gain2, i_gain2, d_gain2, error_gap2, prev_error2, sumError2, goal_pwm2, now_pwm2, pid_control2, 15);

        // // 방향 및 PWM 설정
        // pid_to_pwm1 = (pid_control1/ 100) * 512;
        // pid_to_pwm2 = (pid_control2/ 100) * 512;
        // target_pwm1 = abs(pid_to_pwm1);
        // target_pwm2 = abs(pid_to_pwm2);
        target_pwm1 = abs(pid_control1);
        target_pwm2 = abs(pid_control2);

        // 방향 설정
        direction_flag1 = direction1 ? true : false;
        direction_flag2 = direction2 ? true : false;
        RCLCPP_INFO(rclcpp::get_logger("Motor run"), "Target PWM 1 : %d   ||   Target PWM 2 : %d", static_cast<int>(target_pwm1), static_cast<int>(target_pwm2));
        // 모터 제어
        MotorController(1, direction_flag1, static_cast<int>(target_pwm1));
        MotorController(2, direction_flag2, static_cast<int>(target_pwm2));

        // 모터 정보 출력
        InfoMotors();

        // 오도메트리 계산
        CalculateOdom(dt, delta_linear, delta_angular);

        // 오도메트리 발행
        PublishOdom(delta_linear, delta_angular);

        // RPM 발행
        PublishPWM();
    }

    void
    PublishOdom(double delta_linear, double delta_angular)
    {
        auto odom_msg = std::make_shared<nav_msgs::msg::Odometry>();
        odom_msg->header.stamp = this->now();
        odom_msg->header.frame_id = "odom";
        odom_msg->child_frame_id = "base_footprint";

        // 위치
        odom_msg->pose.pose.position.x = x;
        odom_msg->pose.pose.position.y = y;
        odom_msg->pose.pose.position.z = 0.0;

        // 방향
        tf2::Quaternion q;
        q.setRPY(0, 0, heading);
        odom_msg->pose.pose.orientation = tf2::toMsg(q);

        odom_msg->twist.twist.linear.x = delta_linear;
        odom_msg->twist.twist.angular.z = delta_angular;

        odom_pub_->publish(*odom_msg);

        // Transform Broadcaster
        geometry_msgs::msg::TransformStamped transform;
        transform.header.stamp = this->now();
        transform.header.frame_id = "odom";
        transform.child_frame_id = "base_footprint";
        transform.transform.translation.x = x;
        transform.transform.translation.y = y;
        transform.transform.translation.z = 0.0;
        transform.transform.rotation = odom_msg->pose.pose.orientation;

        tf_broadcaster_->sendTransform(transform);
    }

    void PublishPWM() // pwm 발행
    {
        std_msgs::msg::Float64MultiArray msg;
        msg.data = {
            now_pwm1,
            now_pwm2,
            goal_pwm1,
            goal_pwm2};
        pwm_pub_->publish(msg);
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MotorRunNode>());
    rclcpp::shutdown();
    return 0;
}