/**
 * @file    mecanum_chassis.c
 * @author  Gemini (Technical Consultant)
 * @brief   Mecanum Wheel Chassis Implementation (Cascaded Control + ICM42688)
 */

#include "mecanum_chassis.h"

Mecanum_Chassis_TypeDef Chassis = {0};
extern volatile float global_pulse_sum;

/* 内部静态函数声明 */
static float  Incremental_PID_Controller(Mecanum_Motor_TypeDef *motor);
static void   Motor_Set_Driver_Hardware(uint8 motor_id, int16 pwm_duty);
static void   PID_Parameters_Init(void);

/**
 * @brief  初始化底盘硬件及串级PID参数
 */
void Mecanum_Chassis_Init(void) 
{
    gpio_init(MOTOR_DIR_FL_1, GPO, 0, GPO_PUSH_PULL); gpio_init(MOTOR_DIR_FL_2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_DIR_RL_1, GPO, 0, GPO_PUSH_PULL); gpio_init(MOTOR_DIR_RL_2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_DIR_FR_1, GPO, 0, GPO_PUSH_PULL); gpio_init(MOTOR_DIR_FR_2, GPO, 0, GPO_PUSH_PULL);
    gpio_init(MOTOR_DIR_RR_1, GPO, 0, GPO_PUSH_PULL); gpio_init(MOTOR_DIR_RR_2, GPO, 0, GPO_PUSH_PULL);

    pwm_init(MOTOR_PWM_FL, MAX_PWM_DUTY, 0); pwm_init(MOTOR_PWM_RL, MAX_PWM_DUTY, 0);
    pwm_init(MOTOR_PWM_FR, MAX_PWM_DUTY, 0); pwm_init(MOTOR_PWM_RR, MAX_PWM_DUTY, 0);

    encoder_quad_init(ENC_MODULE_FL, ENC_PIN_FL_A, ENC_PIN_FL_B);
    encoder_quad_init(ENC_MODULE_FR, ENC_PIN_FR_A, ENC_PIN_FR_B);
    encoder_quad_init(ENC_MODULE_RL, ENC_PIN_RL_A, ENC_PIN_RL_B);
    encoder_quad_init(ENC_MODULE_RR, ENC_PIN_RR_A, ENC_PIN_RR_B);

    PID_Parameters_Init();
}

/**
 * @brief  参数初始化：外环 PD，内环 PID + 前馈
 */
static void PID_Parameters_Init(void)
{
    /* 1. 外环 (航向姿态) 参数 */
    Chassis.yaw_pid.kp = 9.0f;   // Kp 视情况微调，纠偏力度
    Chassis.yaw_pid.kd = 2.0f;   // 阻尼系数
    Chassis.yaw_pid.yaw_offset = 0.0f; 
    Chassis.yaw_pid.target_yaw = 0.0f; 

    /* 2. 内环 (单轮速度) 参数 */
    float init_kf = 21.2f;  //21.2f
    float init_kp = 8.0f;   
    float init_ki = 0.5f;   
    float init_kd = 0.0f;

    Chassis.wheel_FL.kp = init_kp; Chassis.wheel_FL.ki = init_ki; Chassis.wheel_FL.kd = init_kd; Chassis.wheel_FL.kf = init_kf;
    Chassis.wheel_FR.kp = init_kp; Chassis.wheel_FR.ki = init_ki; Chassis.wheel_FR.kd = init_kd; Chassis.wheel_FR.kf = init_kf;
    Chassis.wheel_RL.kp = init_kp; Chassis.wheel_RL.ki = init_ki; Chassis.wheel_RL.kd = init_kd; Chassis.wheel_RL.kf = init_kf;
    Chassis.wheel_RR.kp = init_kp; Chassis.wheel_RR.ki = init_ki; Chassis.wheel_RR.kd = init_kd; Chassis.wheel_RR.kf = init_kf;
}

/**
 * @brief  ?? 抓取当前 42688 航向角作为系统的“绝对正前方(0度)”
 * @note   务必在 AHRS 解算稳定后（开机静置 2~3 秒后）调用一次！
 */
void Mecanum_Chassis_Calibrate_Yaw_Zero(void)
{
    Chassis.yaw_pid.yaw_offset = euler_angle_42688.yaw;
    Chassis.yaw_pid.target_yaw = 0.0f; // 归零后，目标就是保持 0 度
}

/**
 * @brief  [串级控制入口] 结合 ICM42688 航向角锁定的全向运动指令
 */
void Mecanum_Set_Velocity_With_IMU(float vx, float vy, float user_wz)
{
    float final_wz = 0.0f;

    float relative_yaw = euler_angle_42688.yaw - Chassis.yaw_pid.yaw_offset;
    if (relative_yaw >  180.0f) relative_yaw -= 360.0f;
    if (relative_yaw < -180.0f) relative_yaw += 360.0f;

    if (user_wz != 0.0f) 
    {
        Chassis.yaw_pid.target_yaw = relative_yaw;
        final_wz = user_wz; 
    }
    else 
    {
        Chassis.yaw_pid.current_yaw = relative_yaw;
        Chassis.yaw_pid.error = Chassis.yaw_pid.target_yaw - relative_yaw;
        
        if (Chassis.yaw_pid.error >  180.0f) Chassis.yaw_pid.error -= 360.0f;
        if (Chassis.yaw_pid.error < -180.0f) Chassis.yaw_pid.error += 360.0f;
        
        float p_term = Chassis.yaw_pid.kp * Chassis.yaw_pid.error;
        float d_term = Chassis.yaw_pid.kd * (-gICM42688Data.gyro.z); 
        
        // ?? 【核心修复】：加负号强制取反！让补偿方向与偏航方向相反！
        Chassis.yaw_pid.compensate_wz = (p_term + d_term);
        
        Chassis.yaw_pid.last_error = Chassis.yaw_pid.error;
        
        if (Chassis.yaw_pid.compensate_wz >  150.0f) Chassis.yaw_pid.compensate_wz =  150.0f;
        if (Chassis.yaw_pid.compensate_wz < -150.0f) Chassis.yaw_pid.compensate_wz = -150.0f;

        final_wz = Chassis.yaw_pid.compensate_wz;
    }

    Mecanum_Set_Velocity(vx, vy, final_wz);
}
/**
 * @brief  [内环核心] 纯逆运动学解算与极限缩放
 */
void Mecanum_Set_Velocity(float vx, float vy, float wz) 
{
    Chassis.vx = vx; Chassis.vy = vy; Chassis.wz = wz;
float v_FL = vx + vy + wz;
    float v_FR = vx - vy - wz;
    float v_RL = vx - vy + wz;
    float v_RR = vx + vy - wz;
//    float v_FL = vx + vy + wz * MECANUM_KINEMATIC_K;
//    float v_FR = vx - vy - wz * MECANUM_KINEMATIC_K;
//    float v_RL = vx - vy + wz * MECANUM_KINEMATIC_K;
//    float v_RR = vx + vy - wz * MECANUM_KINEMATIC_K;

    float max_v = ABS(v_FL);
    if (ABS(v_FR) > max_v) max_v = ABS(v_FR);
    if (ABS(v_RL) > max_v) max_v = ABS(v_RL);
    if (ABS(v_RR) > max_v) max_v = ABS(v_RR);

    if (max_v > MAX_WHEEL_SPEED) 
    {
        float scale_k = MAX_WHEEL_SPEED / max_v; 
        v_FL *= scale_k; v_FR *= scale_k; v_RL *= scale_k; v_RR *= scale_k;
    }

    Chassis.wheel_FL.target_velocity = v_FL;
    Chassis.wheel_FR.target_velocity = v_FR;
    Chassis.wheel_RL.target_velocity = v_RL;
    Chassis.wheel_RR.target_velocity = v_RR;
}

/**
 * @brief  硬件死循环更新任务
 */

// 找到你的 Mecanum_Control_Task 函数，按下面代码修改：
void Mecanum_Control_Task(void) 
{
    Chassis.wheel_FL.current_velocity = (float)encoder_get_count(ENC_MODULE_FL);
    Chassis.wheel_FR.current_velocity = -(float)encoder_get_count(ENC_MODULE_FR); 
    Chassis.wheel_RL.current_velocity = (float)encoder_get_count(ENC_MODULE_RL);  
    Chassis.wheel_RR.current_velocity = -(float)encoder_get_count(ENC_MODULE_RR); 

    encoder_clear_count(ENC_MODULE_FL); encoder_clear_count(ENC_MODULE_FR);
    encoder_clear_count(ENC_MODULE_RL); encoder_clear_count(ENC_MODULE_RR);

    // 【新增核心代码】：累加底盘移动的绝对脉冲数
    // 取四个轮子脉冲绝对值的平均值，代表底盘在当前周期移动的真实距离脉冲
    float current_step_pulse = (ABS(Chassis.wheel_FL.current_velocity) + 
                                ABS(Chassis.wheel_FR.current_velocity) + 
                                ABS(Chassis.wheel_RL.current_velocity) + 
                                ABS(Chassis.wheel_RR.current_velocity)) / 4.0f;
    global_pulse_sum += current_step_pulse; // 累加到全局计步器

    // ... 下方原有的 Incremental_PID_Controller 等代码保持不变 ...
    Incremental_PID_Controller(&Chassis.wheel_FL);
    Incremental_PID_Controller(&Chassis.wheel_FR);
    Incremental_PID_Controller(&Chassis.wheel_RL);
    Incremental_PID_Controller(&Chassis.wheel_RR);

    Motor_Set_Driver_Hardware(1, (int16)Chassis.wheel_FL.output_pwm);
    Motor_Set_Driver_Hardware(2, (int16)Chassis.wheel_FR.output_pwm); 
    Motor_Set_Driver_Hardware(3, (int16)Chassis.wheel_RL.output_pwm);
    Motor_Set_Driver_Hardware(4, (int16)Chassis.wheel_RR.output_pwm);
}

/**
 * @brief  内环位置式 PID + 前馈
 */
static float Incremental_PID_Controller(Mecanum_Motor_TypeDef *motor) 
{
    // ?? 【核心修复】：彻底删除了 target_velocity == 0.0f 时的 return 0 休眠逻辑！
    // 现在哪怕目标速度是 0，电机也会像肌肉一样紧紧绷住！

    motor->error = motor->target_velocity - motor->current_velocity;
    
    // 积分累加
    motor->integral += motor->error;
    
    // 放宽抗饱和限幅，给足破除摩擦力的 PWM 空间
    if (motor->integral >  4000.0f) motor->integral =  4000.0f;
    if (motor->integral < -4000.0f) motor->integral = -4000.0f;

    float ff_term = motor->kf * motor->target_velocity;
    float p_term  = motor->kp * motor->error;
    float i_term  = motor->ki * motor->integral;
    float d_term  = motor->kd * (motor->error - motor->last_error);

    motor->output_pwm = ff_term + p_term + i_term + d_term;
    motor->last_error = motor->error;
    
    if (motor->output_pwm > MAX_PWM_DUTY)  { motor->output_pwm = MAX_PWM_DUTY; }
    if (motor->output_pwm < -MAX_PWM_DUTY) { motor->output_pwm = -MAX_PWM_DUTY; }
    
    return motor->output_pwm;
}

/**
 * @brief  电机底层硬件驱动
 */
static void Motor_Set_Driver_Hardware(uint8 motor_id, int16 pwm_duty) 
{
    uint8 dir1 = 0, dir2 = 0; uint32 abs_pwm = 0;
    if (motor_id == 2 || motor_id == 4) pwm_duty = -pwm_duty;

    if (pwm_duty >= 0) { dir1 = 1; dir2 = 0; abs_pwm = (uint32)pwm_duty; } 
    else               { dir1 = 0; dir2 = 1; abs_pwm = (uint32)(-pwm_duty); }

    switch (motor_id) {
        case 1: gpio_set_level(MOTOR_DIR_FL_1, dir1); gpio_set_level(MOTOR_DIR_FL_2, dir2); pwm_set_duty(MOTOR_PWM_FL, abs_pwm); break;
        case 2: gpio_set_level(MOTOR_DIR_FR_1, dir1); gpio_set_level(MOTOR_DIR_FR_2, dir2); pwm_set_duty(MOTOR_PWM_FR, abs_pwm); break;
        case 3: gpio_set_level(MOTOR_DIR_RL_1, dir1); gpio_set_level(MOTOR_DIR_RL_2, dir2); pwm_set_duty(MOTOR_PWM_RL, abs_pwm); break;
        case 4: gpio_set_level(MOTOR_DIR_RR_1, dir1); gpio_set_level(MOTOR_DIR_RR_2, dir2); pwm_set_duty(MOTOR_PWM_RR, abs_pwm); break;
    }
}