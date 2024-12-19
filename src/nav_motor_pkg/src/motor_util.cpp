#include <nav_motor_pkg/motor_node.hpp>

// 변수 초기 값 정의
// 모터 관련 전역 변수 정의
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
int encoder_resolution = 228;

// 오도메트리 관련 전역 변수 정의
double x = 0.0;
double y = 0.0;
double heading = 0.0;
double left_wheel_old_pos = 0.0;
double right_wheel_old_pos = 0.0;
double left_wheel_new_pos = 0.0;

// 시스템 관련 전역 변수 정의
double control_cycle = 10;
double acceleration_ratio = 25;
double wheel_radius = 0.0575;
double robot_radius = 0.4104;
double wheel_round = 0.0;
double robot_round = 0.0;
bool switch_direction = false;
int theta_distance_flag = 0;

// 모터 핀 번호 정의
int pinum = -1;
int motor1_dir = 19;
int motor2_dir = 6;
int motor1_pwm = 26;
int motor2_pwm = 13;
int motor1_encA = 27;
int motor1_encB = 17;
int motor2_encA = 23;
int motor2_encB = 24;

// PWM 관련 전역 변수 정의
int pwm_frequency = 40000;
int pwm_range = 200;
<<<<<<< HEAD
int pwm_limit = 100;
=======
int pwm_limit = 75;
>>>>>>> 27c8e20 (Maybe Final)

// Interrupt 콜백 함수 선언
void Interrupt1A(int pi, unsigned int gpio, unsigned int level, uint32_t tick);
void Interrupt1B(int pi, unsigned int gpio, unsigned int level, uint32_t tick);
void Interrupt2A(int pi, unsigned int gpio, unsigned int level, uint32_t tick);
void Interrupt2B(int pi, unsigned int gpio, unsigned int level, uint32_t tick);

// 전역 변수
std::deque<double> speed_history_1;
std::deque<double> speed_history_2;
size_t max_window_size = 10; // 저장할 샘플 크기

void LoadParameters() // 파라미터 로드 함수
{
    std::string motor_parameters_path = ament_index_cpp::get_package_share_directory("nav_motor_pkg") + "/config/motor_parameters.txt";
    std::ifstream inFile(motor_parameters_path.c_str());
    if (!inFile.is_open())
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_util"), "Unable to open the file");
        RCLCPP_ERROR_STREAM(rclcpp::get_logger("motor_util"), "file address : " << motor_parameters_path);
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
            pwm_frequency = atof(line.substr(found + 2).c_str());
            break;
        case 1:
            control_cycle = atof(line.substr(found + 2).c_str());
            break;
        case 2:
            acceleration_ratio = atof(line.substr(found + 2).c_str());
            break;
        case 3:
            wheel_radius = atof(line.substr(found + 2).c_str());
            break;
        case 4:
            robot_radius = atof(line.substr(found + 2).c_str());
            break;
        case 5:
            encoder_resolution = atof(line.substr(found + 2).c_str());
            break;
        default:
            break;

            i++;
        }
    }
    inFile.close();
}

int InitMotors() //
{
    pinum = pigpio_start(NULL, NULL);
    if (pinum < 0)
    {
        RCLCPP_ERROR(rclcpp::get_logger("motor_node"), "Pigpio setup failed");
        return 1;
    }

    set_mode(pinum, motor1_dir, PI_OUTPUT);
    set_mode(pinum, motor2_dir, PI_OUTPUT);
    set_mode(pinum, motor1_pwm, PI_OUTPUT);
    set_mode(pinum, motor2_pwm, PI_OUTPUT);
    set_mode(pinum, motor1_encA, PI_INPUT);
    set_mode(pinum, motor1_encB, PI_INPUT);
    set_mode(pinum, motor2_encA, PI_INPUT);
    set_mode(pinum, motor2_encB, PI_INPUT);

    set_PWM_dutycycle(pinum, motor1_pwm, 0);
    set_PWM_dutycycle(pinum, motor2_pwm, 0);

    gpio_write(pinum, motor1_dir, PI_LOW);
    gpio_write(pinum, motor2_dir, PI_LOW);

    set_PWM_range(pinum, motor1_pwm, pwm_range);
    set_PWM_range(pinum, motor2_pwm, pwm_range);

    set_PWM_frequency(pinum, motor1_pwm, pwm_frequency);
    set_PWM_frequency(pinum, motor2_pwm, pwm_frequency);

    set_pull_up_down(pinum, motor1_encA, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor1_encB, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor2_encA, PI_PUD_DOWN);
    set_pull_up_down(pinum, motor2_encB, PI_PUD_DOWN);
<<<<<<< HEAD
    
=======

>>>>>>> 27c8e20 (Maybe Final)
    return 0;
}

void SetInterrupts() // 인터럽트 설정 함수
{
    callback(pinum, motor1_encA, EITHER_EDGE, Interrupt1A);
    callback(pinum, motor1_encB, EITHER_EDGE, Interrupt1B);
    callback(pinum, motor2_encA, EITHER_EDGE, Interrupt2A);
    callback(pinum, motor2_encB, EITHER_EDGE, Interrupt2B);
}

//////////////////////////// Interrupt 콜백 함수 ////////////////////////////
void Interrupt1A(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)gpio;
    (void)level;
    (void)tick;

    if (gpio_read(pinum, motor1_dir))
    {
        encoder_count_1A--;
    }
    else
    {
        encoder_count_1A++;
    }
    speed_count_1++;

    // RCLCPP_INFO(rclcpp::get_logger("Interrupt1A"), "speed_count_1 = %d", speed_count_1);
}

void Interrupt1B(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)gpio;
    (void)level;
    (void)tick;

    if (gpio_read(pinum, motor1_dir))
    {
        encoder_count_1B--;
    }
    else
    {
        encoder_count_1B++;
    }
    speed_count_1++;
}

void Interrupt2A(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)gpio;
    (void)level;
    (void)tick;

    bool current_direction = gpio_read(pinum, motor2_dir);
    if (current_direction)
    {
        encoder_count_2A--;
    }
    else
    {
        encoder_count_2A++;
    }
    speed_count_2++;
    // RCLCPP_INFO(rclcpp::get_logger("Interrupt2A"), "current_direction = %s",current_direction ? "CW" : "CCW");
}

void Interrupt2B(int pi, unsigned gpio, unsigned level, uint32_t tick)
{
    (void)pi;
    (void)gpio;
    (void)level;
    (void)tick;

    if (gpio_read(pinum, motor2_dir))
    {
        encoder_count_2B--;
    }
    else
    {
        encoder_count_2B++;
    }
    speed_count_2++;
}

//////////////////////////// 엔코더 합산 함수 ////////////////////////////

int SumMotor1Encoder()
{
    encoder_count_1 = encoder_count_1A + encoder_count_1B;
    return encoder_count_1;
}

int SumMotor2Encoder()
{
    encoder_count_2 = encoder_count_2A + encoder_count_2B;
    return encoder_count_2;
}

void InitEncoders() // 엔코더 초기화 함수
{

    encoder_count_1 = 0;
    encoder_count_2 = 0;
    encoder_count_1A = 0;
    encoder_count_1B = 0;
    encoder_count_2A = 0;
    encoder_count_2B = 0;
    speed_count_1 = 0;
    speed_count_2 = 0;
}

void Initialize() // 초기화 함수
{
    LoadParameters();
    InitMotors();
    InitEncoders();
    SetInterrupts();

    wheel_round = 2 * M_PI * wheel_radius;
    robot_round = 2 * M_PI * robot_radius;
    switch_direction = true;
    theta_distance_flag = 0;

    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_range %d", pwm_range);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_frequency %d", pwm_frequency);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "pwm_limit %d", pwm_limit);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "control_cycle %f", control_cycle);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "acceleration_ratio %f", acceleration_ratio);
    RCLCPP_INFO(rclcpp::get_logger("motor_node"), "Initialize Complete");
}

int LimitPwm(int &pwm) // pwm 제한 함수
{
    int output;
    if (pwm > pwm_limit * 2)
    {
        output = pwm_limit;
        RCLCPP_WARN(rclcpp::get_logger("motor_node"), "pwm too fast!!!");
    }
    else if (pwm > pwm_limit)
        output = pwm_limit;
    else if (pwm < 0)
    {
        output = 0;
        RCLCPP_WARN(rclcpp::get_logger("motor_node"), "trash value!!!");
    }
    else
    {
        output = pwm;
    }
    pwm = output;
}

void MotorController(
    int motor_num, bool direction, int pwm)
{
    int local_pwm = std::min(std::max(pwm, 0), pwm_range);

    if (motor_num == 1)
    {
        gpio_write(pinum, motor1_dir, direction ? PI_LOW : PI_HIGH);
        set_PWM_dutycycle(pinum, motor1_pwm, local_pwm);
        current_pwm1 = local_pwm;
        current_direction1 = direction;
    }
    else if (motor_num == 2)
    {
        gpio_write(pinum, motor2_dir, direction ? PI_LOW : PI_HIGH);
        set_PWM_dutycycle(pinum, motor2_pwm, local_pwm);
        current_pwm2 = local_pwm;
        current_direction2 = direction;
    }
}

void CalculateRpm(double dt, double &now_rpm1, double &now_rpm2) // RPM 계산 함수
{

    int direction_factor1 = 1, direction_factor2 = 1;

    direction_factor1 = gpio_read(pinum, motor1_dir) ? -1 : 1;
    direction_factor2 = gpio_read(pinum, motor2_dir) ? -1 : 1;
    RCLCPP_INFO(rclcpp::get_logger("Calculate RPM"), "direction_factor1 : %d     ||     direction_factor2 : %d\n", direction_factor1, direction_factor2);

    dt = dt / 1000.0; // ms로 변환

    rpm_value1 = direction_factor1 * (speed_count_1 * 60) / (encoder_resolution * 4 * dt);
    rpm_value2 = direction_factor2 * (speed_count_2 * 60) / (encoder_resolution * 4 * dt);

    // // 슬라이딩 윈도우에 추가
    // speed_history_1.push_back(rpm1);
    // speed_history_2.push_back(rpm2);

    // // 최대 크기 초과 시 가장 오래된 값 제거
    // if (speed_history_1.size() > max_window_size)
    //     speed_history_1.pop_front();
    // if (speed_history_2.size() > max_window_size)
    //     speed_history_2.pop_front();

    // // 평균 계산s
    // rpm_value1 = std::accumulate(speed_history_1.begin(), speed_history_1.end(), 0.0) / speed_history_1.size();
    // rpm_value2 = std::accumulate(speed_history_2.begin(), speed_history_2.end(), 0.0) / speed_history_2.size();

    now_rpm1 = rpm_value1;
    now_rpm2 = rpm_value2;

    // 각 주기마다 speed_count 초기화
    speed_count_1 = 0;
    speed_count_2 = 0;
}

void InfoMotors() // 모터 정보 출력 함수
{
    // printf("\033[2J");
    // printf("\033[1;1H");
    printf("Encoder1A : %5d    ||  Encoder2A : %5d\n", encoder_count_1A, encoder_count_2A);
    printf("Encoder1B : %5d    ||  Encoder2B : %5d\n", encoder_count_1B, encoder_count_2B);
    printf("RPM1 : %10.0f    ||  RPM2 : %10.0f\n", rpm_value1, rpm_value2);
    printf("PWM1 : %10.0f    ||  PWM2 : %10.0f\n", current_pwm1, current_pwm2);
    printf("DIR1 :%11s    ||  DIR2 :%11s\n", current_direction1 ? "CW" : "CCW", current_direction2 ? "CW" : "CCW");
    printf("\n");
}

<<<<<<< HEAD
void PidController(double p_gain1, double i_gain1, double d_gain1, double errorGap1, double &prevError1, double &sumError1, double target_rpm1, double current_rpm1, double time_interval, double &filteredPidControl1,
                   double p_gain2, double i_gain2, double d_gain2, double errorGap2, double &prevError2, double &sumError2, double target_rpm2, double current_rpm2, double &filteredPidControl2) // PID 제어 함수
{
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"), "target_rpm1:%10.0f   ||   target_rpm2:%10.0f", target_rpm1, target_rpm2);
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"), "current_rpm1:%10.0f   ||   current_rpm2:%10.0f", current_rpm1, current_rpm2);

    // Error 계산
    errorGap1 = target_rpm1 - current_rpm1;
    errorGap2 = target_rpm2 - current_rpm2;
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"), "ErrorGap1:%10.0f   ||   ErrorGap2:%10.0f", errorGap1, errorGap2);

    // 에러 적분값 누적
    sumError1 += errorGap1 * time_interval;
    sumError2 += errorGap2 * time_interval;

    // P, I, D 제어 계산
=======

void PidController(double p_gain1, double i_gain1, double d_gain1, double errorGap1, double &prevError1, double &sumError1, double target_rpm1, double current_rpm1, double time_interval, double &filteredPidControl1,
                    double p_gain2, double i_gain2, double d_gain2, double errorGap2, double &prevError2, double &sumError2, double target_rpm2, double current_rpm2, double &filteredPidControl2, double max_spike_threshold)
{
    static double prev_rpm1 = 0.0;
    static double prev_rpm2 = 0.0;

    // 스파이크 필터
    
    if (std::abs(current_rpm1 - prev_rpm1) > max_spike_threshold)
    {
        current_rpm1 = prev_rpm1; // 튀는 값 무시
    }
    if (std::abs(current_rpm2 - prev_rpm2) > max_spike_threshold)
    {
        current_rpm2 = prev_rpm2;
    }

    // 로우 패스 필터
    double alpha_rpm = 0.3;
    current_rpm1 = alpha_rpm * current_rpm1 + (1.0 - alpha_rpm) * prev_rpm1;
    current_rpm2 = alpha_rpm * current_rpm2 + (1.0 - alpha_rpm) * prev_rpm2;
    
    // 작게 튀는 값 무시
    const double deadband = 1.0;
    errorGap1 = std::abs(target_rpm1 - current_rpm1) < deadband ? 0.0 : target_rpm1 - current_rpm1;
    errorGap2 = std::abs(target_rpm2 - current_rpm2) < deadband ? 0.0 : target_rpm2 - current_rpm2;

>>>>>>> 27c8e20 (Maybe Final)
    double pControl1 = p_gain1 * errorGap1;
    double iControl1 = i_gain1 * sumError1;
    double dControl1 = d_gain1 * (errorGap1 - prevError1) / time_interval;

    double pControl2 = p_gain2 * errorGap2;
    double iControl2 = i_gain2 * sumError2;
    double dControl2 = d_gain2 * (errorGap2 - prevError2) / time_interval;

<<<<<<< HEAD
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"), "pControl1:%10.0f   ||   pControl2:%10.0f", pControl1, pControl2);
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"), "iControl1:%10.0f   ||   iControl2:%10.0f", iControl1, iControl2);
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"), "dControl1:%10.0f   ||   dControl2:%10.0f", dControl1, dControl2);

    // PID 합산
    double pidControl1 = pControl1 + iControl1 + dControl1;
    double pidControl2 = pControl2 + iControl2 + dControl2;
    RCLCPP_INFO(rclcpp::get_logger("pidCotroller"), "pidControl1:%10.0f   ||   pidControl2:%10.0f", pidControl1, pidControl2);

    // PID 제어값 필터링 (저역 통과 필터 적용 가능)
    double alpha = 0.4; // 필터 강도 (0.0 ~ 1.0 사이 값)
    filteredPidControl1 = alpha * pidControl1 + (1.0 - alpha) * filteredPidControl1;
    filteredPidControl2 = alpha * pidControl2 + (1.0 - alpha) * filteredPidControl2;

    // 이전 에러값 업데이트
    prevError1 = errorGap1;
    prevError2 = errorGap2;
}

=======
    filteredPidControl1 = pControl1 + iControl1 + dControl1;
    filteredPidControl2 = pControl2 + iControl2 + dControl2;

    // // PID 출력 필터링 
    // double alpha = 0.3;
    // filteredPidControl1 = alpha * pidControl1 + (1.0 - alpha) * filteredPidControl1;
    // filteredPidControl2 = alpha * pidControl2 + (1.0 - alpha) * filteredPidControl2;

    // 이전 값 업데이트
    prevError1 = errorGap1;
    prevError2 = errorGap2;
    prev_rpm1 = current_rpm1;
    prev_rpm2 = current_rpm2;
}


>>>>>>> 27c8e20 (Maybe Final)
void CalculateOdom(double dt, double &delta_linear, double &delta_angular) // 오도메트리 계산 함수
{
    int pulse_1 = SumMotor1Encoder();
    int pulse_2 = SumMotor2Encoder();

    double now_left_wheel_pose = wheel_radius * (pulse_1 * 2 * M_PI) / (4 * encoder_resolution);
    double now_right_wheel_pose = wheel_radius * (pulse_2 * 2 * M_PI) / (4 * encoder_resolution);

    double left_vel = (now_left_wheel_pose - left_wheel_old_pos);
<<<<<<< HEAD
    double right_vel = -1 * (now_right_wheel_pose - right_wheel_old_pos);
=======
    double right_vel = (now_right_wheel_pose - right_wheel_old_pos);
>>>>>>> 27c8e20 (Maybe Final)

    left_wheel_old_pos = now_left_wheel_pose;
    right_wheel_old_pos = now_right_wheel_pose;

<<<<<<< HEAD
    double delta_distance = (left_vel + right_vel) / 2.0;
    double delta_theta = (right_vel - left_vel) / robot_radius;
=======
    double delta_distance = (left_vel + (-1 * right_vel)) / 2.0;
    double delta_theta = (left_vel - (-1 * right_vel)) / robot_radius;
>>>>>>> 27c8e20 (Maybe Final)

    x += delta_distance * cos(heading);
    y += delta_distance * sin(heading);
    heading += delta_theta;

    delta_linear = delta_distance / dt;
    delta_angular = delta_theta / dt;
}

void calculateWheelPWM(double linear_velocity, double angular_velocity, double &pwm_left, double &pwm_right)
{

    // 양쪽 바퀴 속도 계산
    double v_left = linear_velocity - (angular_velocity * robot_radius / 2.0);  // 왼쪽 바퀴 선속도 (m/s)
    double v_right = linear_velocity + (angular_velocity * robot_radius / 2.0); // 오른쪽 바퀴 선속도 (m/s)

    // 바퀴 속도를 PWM 값으로 변환
    const double max_velocity = 10; // 최대 바퀴 속도 (m/s) (예: 1m/s)
    const double max_pwm = 150;     // 최대 PWM 값

    // pwm_left = std::clamp((v_left / max_velocity) * max_pwm, -max_pwm, max_pwm);  // -255 ~ 255로 제한
    // pwm_right = -1 * std::clamp((v_right / max_velocity) * max_pwm, -max_pwm, max_pwm); // -255 ~ 255로 제한
    const double conversion_factor = 60.0 / (2.0 * M_PI); // 초당 회전을 분당 회전으로 변환

    pwm_left = std::clamp(((v_left / wheel_radius) * conversion_factor), -max_pwm, max_pwm);
    pwm_right = -1 * std::clamp(((v_right / wheel_radius) * conversion_factor), -max_pwm, max_pwm);

    pwm_left = (pwm_left / 100) * 150;
    pwm_right = (pwm_right / 100) * 150;
}