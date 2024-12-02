#include <crash_motor/motor_node.hpp>

// Interrupt 콜백 함수 선언
void Interrupt1A(int pi, unsigned int gpio, unsigned int level, uint32_t tick);
void Interrupt1B(int pi, unsigned int gpio, unsigned int level, uint32_t tick);
void Interrupt2A(int pi, unsigned int gpio, unsigned int level, uint32_t tick);
void Interrupt2B(int pi, unsigned int gpio, unsigned int level, uint32_t tick);

// 전역 객체 선언
SystemState global_system_state;

// 전역 변수
std::deque<double> speed_history_1;
std::deque<double> speed_history_2;
size_t max_window_size = 10; // 저장할 샘플 크기
std::deque<double> pid_history_1;
std::deque<double> pid_history_2;

// 예제 및 기본 코드

void LoadParameters()
{
    auto &system_state = global_system_state;
    std::string motor_parameters_path = ament_index_cpp::get_package_share_directory("crash_motor") + "/config/motor_parameters.txt";
    std::ifstream inFile(motor_parameters_path.c_str());
    if (!inFile.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_test_node"), "Unable to open the file");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("motor_test_node"), "file address : " << motor_parameters_path);
        return;
    }

    int i = 0;
    std::size_t found;
    for (std::string line; std::getline(inFile, line);)
    {
        found = line.find("=");
        switch (i)
        {
        case 0:
            system_state.pwm_frequency = atof(line.substr(found + 2).c_str());
            break;
        case 1:
            system_state.control_cycle = atof(line.substr(found + 2).c_str());
            break;
        case 2:
            system_state.acceleration_ratio = atof(line.substr(found + 2).c_str());
            break;
        case 3:
            system_state.wheel_radius = atof(line.substr(found + 2).c_str());
            break;
        case 4:
            system_state.robot_radius = atof(line.substr(found + 2).c_str());
            break;
        case 5:
            system_state.encoder_resolution = atof(line.substr(found + 2).c_str());
            break;
        default:
            break;

            i++;
        }
    }
    inFile.close();
}

int InitMotors()
{
    auto &system_state = global_system_state;
    system_state.pinum = pigpio_start(NULL, NULL);
    if (system_state.pinum < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Pigpio setup failed");
        return 1;
    }

    set_mode(system_state.pinum, system_state.motor1_dir, PI_OUTPUT);
    set_mode(system_state.pinum, system_state.motor2_dir, PI_OUTPUT);
    set_mode(system_state.pinum, system_state.motor1_pwm, PI_OUTPUT);
    set_mode(system_state.pinum, system_state.motor2_pwm, PI_OUTPUT);
    set_mode(system_state.pinum, system_state.motor1_encA, PI_INPUT);
    set_mode(system_state.pinum, system_state.motor1_encB, PI_INPUT);
    set_mode(system_state.pinum, system_state.motor2_encA, PI_INPUT);
    set_mode(system_state.pinum, system_state.motor2_encB, PI_INPUT);

    gpio_write(system_state.pinum, system_state.motor1_dir, PI_LOW);
    gpio_write(system_state.pinum, system_state.motor2_dir, PI_LOW);

    set_PWM_range(system_state.pinum, system_state.motor1_pwm, system_state.pwm_range);
    set_PWM_frequency(system_state.pinum, system_state.motor1_pwm, system_state.pwm_frequency);
    set_PWM_range(system_state.pinum, system_state.motor2_pwm, system_state.pwm_range);
    set_PWM_frequency(system_state.pinum, system_state.motor2_pwm, system_state.pwm_frequency);

    set_PWM_dutycycle(system_state.pinum, system_state.motor1_pwm, 0);
    set_PWM_dutycycle(system_state.pinum, system_state.motor2_pwm, 0);

    set_pull_up_down(system_state.pinum, system_state.motor1_encA, PI_PUD_DOWN);
    set_pull_up_down(system_state.pinum, system_state.motor1_encB, PI_PUD_DOWN);
    set_pull_up_down(system_state.pinum, system_state.motor2_encA, PI_PUD_DOWN);
    set_pull_up_down(system_state.pinum, system_state.motor2_encB, PI_PUD_DOWN);
    return 0;
}

void SetInterrupts()
{
    auto &system_state = global_system_state;
    callback(system_state.pinum, system_state.motor1_encA, EITHER_EDGE, Interrupt1A);
    callback(system_state.pinum, system_state.motor1_encB, EITHER_EDGE, Interrupt1B);
    callback(system_state.pinum, system_state.motor2_encA, EITHER_EDGE, Interrupt2A);
    callback(system_state.pinum, system_state.motor2_encB, EITHER_EDGE, Interrupt2B);
}

void Interrupt1A(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)gpio;
    (void)level;
    (void)tick;
    
    auto &system_state = global_system_state;
    auto &motor_state = global_system_state.motor_state;
    if (gpio_read(system_state.pinum, global_system_state.motor1_dir))
    {
        motor_state.encoder_count_1A--;
    }
    else
    {
        motor_state.encoder_count_1A++;
    }
    motor_state.speed_count_1++;
   
    // RCLCPP_INFO(rclcpp::get_logger("Interrupt1A"), "speed_count_1 = %d", motor_state.speed_count_1);
}

void Interrupt1B(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)gpio;
    (void)level;
    (void)tick;

    auto &system_state = global_system_state;
    auto &motor_state = global_system_state.motor_state;
    if (gpio_read(system_state.pinum, global_system_state.motor1_dir))
    {
        motor_state.encoder_count_1B--;
    }
    else
    {
        motor_state.encoder_count_1B++;
    }
    motor_state.speed_count_1++;
}

void Interrupt2A(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)gpio;
    (void)level;
    (void)tick;

    auto &system_state = global_system_state;
    auto &motor_state = global_system_state.motor_state;
    bool current_direction = gpio_read(system_state.pinum, global_system_state.motor2_dir);
    if (current_direction)
    {
        motor_state.encoder_count_2A--;
    }
    else
    {
        motor_state.encoder_count_2A++;
    }
    motor_state.speed_count_2++;
    // RCLCPP_INFO(rclcpp::get_logger("Interrupt2A"), "current_direction = %s",current_direction ? "CW" : "CCW");
}

void Interrupt2B(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)gpio;
    (void)level;
    (void)tick;
    
    auto &system_state = global_system_state;
    auto &motor_state = global_system_state.motor_state;
    if (gpio_read(system_state.pinum, global_system_state.motor2_dir))
    {
        motor_state.encoder_count_2B--;
    }
    else
    {
        motor_state.encoder_count_2B++;
    }
    motor_state.speed_count_2++;
}

int SumMotor1Encoder()
{   
    auto &motor_state = global_system_state.motor_state;
    motor_state.encoder_count_1 = motor_state.encoder_count_1A + motor_state.encoder_count_1B;
    return motor_state.encoder_count_1;
}

int SumMotor2Encoder()
{
    auto &motor_state = global_system_state.motor_state;
    motor_state.encoder_count_2 = motor_state.encoder_count_2A + motor_state.encoder_count_2B;
    return motor_state.encoder_count_2;
}

void InitEncoders()
{
    auto &motor_state = global_system_state.motor_state;
    
    motor_state.encoder_count_1 = 0;
    motor_state.encoder_count_2 = 0;
    motor_state.encoder_count_1A = 0;
    motor_state.encoder_count_1B = 0;
    motor_state.encoder_count_2A = 0;
    motor_state.encoder_count_2B = 0;
    motor_state.speed_count_1 = 0;
    motor_state.speed_count_2 = 0;
}

//////////////////////////////////////////////////////////////////////////////////////////////////////////
void Initialize()
{
    auto &system_state = global_system_state;
    LoadParameters();
    InitMotors();
    InitEncoders();
    SetInterrupts();

    system_state.wheel_round = 2 * M_PI * system_state.wheel_radius;
    system_state.robot_round = 2 * M_PI * system_state.robot_radius;
    system_state.switch_direction = true;
    system_state.theta_distance_flag = 0;

    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_range %d", system_state.pwm_range);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_frequency %d", system_state.pwm_frequency);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_limit %d", system_state.pwm_limit);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "control_cycle %f", system_state.control_cycle);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "acceleration_ratio %f", system_state.acceleration_ratio);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Initialize Complete");
}

void MotorController(
    int motor_num,
    bool direction,
    int pwm
    )
{
    auto &system_state = global_system_state;
    auto &motor_state = global_system_state.motor_state;
    int pinum = system_state.pinum;
    int local_pwm = std::min(std::max(pwm, 0), system_state.pwm_range);

    if (motor_num == 1)
    {
        // std::cout << "AAA : " << direction << std::endl;
        gpio_write(pinum, system_state.motor1_dir, direction ? PI_LOW : PI_HIGH);
        set_PWM_dutycycle(pinum, system_state.motor1_pwm, local_pwm);
        motor_state.current_pwm1 = local_pwm;
        motor_state.current_direction1 = direction;
    }
    else if (motor_num == 2)
    {
        gpio_write(pinum, system_state.motor2_dir, direction ? PI_LOW : PI_HIGH);
        set_PWM_dutycycle(pinum, system_state.motor2_pwm, local_pwm);
        motor_state.current_pwm2 = local_pwm;
        motor_state.current_direction2 = direction;
    }
}

void AccelController(
    int motor_num,
    bool direction,
    int desired_pwm)
{
    bool local_current_direction;
    int local_current_pwm;
    auto &system_state = global_system_state;
    auto &motor_state = global_system_state.motor_state;

    if (motor_num == 1)
    {
        local_current_direction = motor_state.current_direction1;
        local_current_pwm = motor_state.current_pwm1;
    }
    else if (motor_num == 2)
    {
        local_current_direction = motor_state.current_direction2;
        local_current_pwm = motor_state.current_pwm2;
    }

    int acceleration = static_cast<int>(system_state.pwm_range / system_state.acceleration_ratio);
    int local_pwm;

    if (direction == local_current_direction)
    {
        local_pwm = std::clamp(local_current_pwm + (desired_pwm > local_current_pwm ? acceleration : -acceleration), 0, desired_pwm);
    }
    else
    {
        local_pwm = local_current_pwm - acceleration;
        if (local_pwm <= 0)
        {
            MotorController(motor_num, direction, 0);
            return;
        }
    }

    MotorController(motor_num, direction, local_pwm);
}

void SwitchTurn(SystemState &state, int pwm1, int pwm2, bool &switch_direction)
{
    int local_pwm1 = LimitPwm(pwm1);
    int local_pwm2 = LimitPwm(pwm2);

    if (switch_direction)
    {
        MotorController(1, true, local_pwm1);
        MotorController(2, true, local_pwm2);
        switch_direction = false;
        RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Switch Direction: true");
    }
    else
    {
        MotorController(1, false, local_pwm1);
        MotorController(2, false, local_pwm2);
        switch_direction = true;
        RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Switch Direction: false");
    }
}

void ThetaTurn(double theta, int pwm, SystemState &state)
{
    double local_encoder;
    int local_pwm = LimitPwm(pwm);
    auto &motor_state = state.motor_state;

    int encoder_count_1 = SumMotor1Encoder();

    if (theta > 0)
    {
        local_encoder = (state.encoder_resolution * 4 / 360.0) *
                        (state.robot_round / state.wheel_round) * theta;
        MotorController(1, true, local_pwm);
        MotorController(2, true, local_pwm);
    }
    else
    {
        local_encoder = -(state.encoder_resolution * 4 / 360.0) *
                        (state.robot_round / state.wheel_round) * theta;
        MotorController(1, false, local_pwm);
        MotorController(2, false, local_pwm);
    }

    if (encoder_count_1 >= local_encoder)
    {
        InitEncoders();
        MotorController(1, true, 0);
        MotorController(2, true, 0);
    }
}

void DistanceGo(double distance, int pwm, SystemState &state)
{
    auto &motor_state = state.motor_state;
    int encoder_count_1 = SumMotor1Encoder();

    double local_encoder = (state.encoder_resolution * 4 * std::abs(distance)) /
                           state.wheel_round;
    bool direction = distance > 0;
    int local_pwm = LimitPwm(pwm);

    if (encoder_count_1 < local_encoder)
    {
        MotorController( 1, direction, local_pwm);
        MotorController(2, !direction, local_pwm);
    }
    else
    {
        InitEncoders();
        MotorController(1, true, 0);
        MotorController(2, true, 0);
    }
}

void ThetaTurnDistanceGo(double theta, int turn_pwm, double distance, int go_pwm, SystemState &state)
{
    if (state.theta_distance_flag == 0)
    {
        state.theta_distance_flag = 1; // 회전 시작
    }
    else if (state.theta_distance_flag == 1 || state.theta_distance_flag == 2)
    {
        ThetaTurn(theta, turn_pwm, state);
        state.theta_distance_flag = 3; // 직진 준비
    }
    else if (state.theta_distance_flag == 3 || state.theta_distance_flag == 4)
    {
        DistanceGo(distance, go_pwm, state);
        state.theta_distance_flag = 0; // 동작 완료
    }
}

int LimitPwm(int pwm)
{   
    auto &state = global_system_state;
    int output;
    if (pwm > state.pwm_limit*2)
    {
        output = state.pwm_limit;
        RCLCPP_WARN(rclcpp::get_logger("motor_node"), "pwm too fast!!!");
    }
    else if (pwm > state.pwm_limit)
        output = state.pwm_limit;
    else if (pwm < 0)
    {
        output = 0;
        RCLCPP_WARN(rclcpp::get_logger("motor_node"), "trash value!!!");
    }
    else
    {
        output = pwm;
    }
    return output;
}

void CalculateRpm(double dt, double &now_rpm1, double &now_rpm2)
{   
    auto &state_ = global_system_state;
    auto &motor_state = global_system_state.motor_state;
    double rpm1 = 0.0, rpm2 = 0.0;
    int direction_factor1 = 1, direction_factor2 = 1;

    direction_factor1 = gpio_read(state_.pinum, state_.motor1_dir) ? 1 : -1;
    direction_factor2 = gpio_read(state_.pinum, state_.motor2_dir) ? 1 : -1;
    // RCLCPP_INFO(rclcpp::get_logger("CalculateRpm"),"speed_count_1: %d",motor_state.speed_count_1);
    // motor_state.rpm_value1 = (motor_state.speed_count_1 * 0.000986);
    // motor_state.speed_count_1 = 0;
    // RCLCPP_INFO(rclcpp::get_logger("CalculateRpm"),"rpm_value1:%10.0f",motor_state.rpm_value1);
    // motor_state.rpm_value2 = (motor_state.speed_count_2 * (60.0 * state_.control_cycle)) / (state_.encoder_resolution * 4);
    // RCLCPP_INFO(rclcpp::get_logger("CalculateRpm"),"rpm_value2:%10.0f",motor_state.rpm_value2);
    // motor_state.speed_count_2 = 0;
    // now_rpm1 = motor_state.rpm_value1;
    // now_rpm2 = motor_state.rpm_value2;

    // 현재 speed_count 기반으로 RPM 계산   
    // motor_state.rpm_value1 = direction_factor1 * (motor_state.speed_count_1 * 60) / (dt * state_.encoder_resolution * 4);
    // motor_state.rpm_value2 = direction_factor2 * (motor_state.speed_count_2 * 60) / (dt * state_.encoder_resolution * 4);
    rpm1 = direction_factor1 * (motor_state.speed_count_1 * 60 * state_.control_cycle) / (state_.encoder_resolution * 4);
    rpm2 = direction_factor2 * (motor_state.speed_count_2 * 60 * state_.control_cycle) / (state_.encoder_resolution * 4);
    // 슬라이딩 윈도우에 추가
    speed_history_1.push_back(rpm1);
    speed_history_2.push_back(rpm2);

    // 최대 크기 초과 시 가장 오래된 값 제거
    if (speed_history_1.size() > max_window_size) speed_history_1.pop_front();
    if (speed_history_2.size() > max_window_size) speed_history_2.pop_front();

    // 평균 계산s
    motor_state.rpm_value1 = std::accumulate(speed_history_1.begin(), speed_history_1.end(), 0.0) / speed_history_1.size();
    motor_state.rpm_value2 = std::accumulate(speed_history_2.begin(), speed_history_2.end(), 0.0) / speed_history_2.size();
    
    now_rpm1 = motor_state.rpm_value1;
    now_rpm2 = motor_state.rpm_value2;

    // 각 주기마다 speed_count 초기화
    motor_state.speed_count_1 = 0;
    motor_state.speed_count_2 = 0;
}

void InfoMotors()
{
    auto motor_state = global_system_state.motor_state;

    printf("\033[2J");
    printf("\033[1;1H");
    printf("Encoder1A : %5d    ||  Encoder2A : %5d\n", motor_state.encoder_count_1A, motor_state.encoder_count_2A);
    printf("Encoder1B : %5d    ||  Encoder2B : %5d\n", motor_state.encoder_count_1B, motor_state.encoder_count_2B);
    printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", motor_state.rpm_value1, motor_state.rpm_value2);
    printf("PWM1 : %10.0f    ||  PWM2 : %10.0f\n", motor_state.current_pwm1, motor_state.current_pwm2);
    printf("DIR1 :%11s    ||  DIR2 :%11s\n", motor_state.current_direction1 ? "CW" : "CCW", motor_state.current_direction2 ? "CW" : "CCW");
    printf("\n");
}

void PidController(
    double p_gain1,
    double d_gain1,
    double errorGap1,
    double &prevError1,
    double target_rpm1,
    double current_rpm1,
    double time_interval,
    double &pidControl1,
    double p_gain2,
    double d_gain2,
    double errorGap2,
    double &prevError2,
    double target_rpm2,
    double current_rpm2,
    double &pidControl2) 
{
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"),"target_rpm1:%10.0f   ||   target_rpm2:%10.0f", target_rpm1, target_rpm2);
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"),"current_rpm2:%10.0f   ||   target_rpm2:%10.0f", current_rpm1, current_rpm2);

    errorGap1 = target_rpm1 - current_rpm1;
    errorGap2 = target_rpm2 - current_rpm2;
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"),"ErrorGap1:%10.0f   ||   ErrorGap2:%10.0f", errorGap1, errorGap2);

    double pControl1 = p_gain1 * errorGap1;
    double dControl1 = d_gain1 * (errorGap1 - prevError1) / time_interval;
    double pControl2 = p_gain2 * errorGap2;
    double dControl2 = d_gain2 * (errorGap2 - prevError2) / time_interval;
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"),"pControl1:%10.0f   ||   pControl2:%10.0f", pControl1, pControl2);
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"),"dControl1:%10.0f   ||   dControl2:%10.0f", dControl1, dControl2);

    pidControl1 = pControl1 + dControl1;
    pidControl2 = pControl2 + dControl2;
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"),"pdControl1:%10.0f   ||   pdControl2:%10.0f", pidControl1, pidControl2);

    prevError1 = errorGap1;
    prevError2 = errorGap2;
    //  // 슬라이딩 윈도우에 추가
    // pid_history_1.push_back(time_pidControl1);
    // pid_history_2.push_back(time_pidControl2);

    // // 최대 크기 초과 시 가장 오래된 값 제거
    // if (pid_history_1.size() > max_window_size) pid_history_1.pop_front();
    // if (pid_history_2.size() > max_window_size) pid_history_2.pop_front();

    // // 평균 계산
    // pidControl1 = std::accumulate(pid_history_1.begin(), pid_history_1.end(), 0.0) / pid_history_1.size();
    // pidControl2 = std::accumulate(pid_history_2.begin(), pid_history_2.end(), 0.0) / pid_history_2.size();
}

void CalculateOdom(
    double dt,
    double &delta_linear,
    double &delta_angular)
{
    auto &system_state = global_system_state;
    auto &motor_state = system_state.motor_state;
    auto &odom_state = system_state.odom_state;

    int pulse_1 = SumMotor1Encoder();
    int pulse_2 = SumMotor2Encoder();

    double now_left_wheel_pose = system_state.wheel_radius * (pulse_1 * 2 * M_PI) / (4 * system_state.encoder_resolution);
    double now_right_wheel_pose = system_state.wheel_radius * (pulse_2 * 2 * M_PI) / (4 * system_state.encoder_resolution);

    double left_vel = now_left_wheel_pose - odom_state.left_wheel_old_pos;
    double right_vel = now_right_wheel_pose - odom_state.right_wheel_old_pos;

    odom_state.left_wheel_old_pos = now_left_wheel_pose;
    odom_state.right_wheel_old_pos = now_right_wheel_pose;

    double delta_distance = (left_vel + right_vel) / 2.0;
    double delta_theta = (right_vel - left_vel) / system_state.robot_radius;

    odom_state.x += delta_distance * cos(odom_state.heading);
    odom_state.y += delta_distance * sin(odom_state.heading);
    odom_state.heading += delta_theta;

    delta_linear = delta_distance / dt;
    delta_angular = delta_theta / dt;
}

void oneTimeEncoder(){
    MotorController(1,true,50);
    if (global_system_state.motor_state.encoder_count_1 == 912){
        MotorController(1, true, 0);
    }
    MotorController(2,true,50);
    if (global_system_state.motor_state.encoder_count_2 == 912){
        MotorController(2, true, 0);
    }

    
}