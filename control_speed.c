#include "control_speed.h"

/* 变量定义 */
Speed_PID_t left_speed_pid;
Speed_PID_t right_speed_pid;

/**
 * @brief 速度环参数初始化
 */
void pid_speed_init(void)
{
    // 左轮参数
    left_speed_pid.Kp = 27.4f;
    left_speed_pid.Ki = 0.8f;
    left_speed_pid.Kd = 15.0f;
    left_speed_pid.Out = 0.0f;
    left_speed_pid.Out_Limit = 4000.0f;
    left_speed_pid.Err0 = left_speed_pid.Err1 = left_speed_pid.Err2 = 0.0f;

    // 右轮参数 
    right_speed_pid.Kp = 27.4f;
    right_speed_pid.Ki = 0.8f;
    right_speed_pid.Kd = 15.0f;
    right_speed_pid.Out = 0.0f;
    right_speed_pid.Out_Limit = 4000.0f;
    right_speed_pid.Err0 = right_speed_pid.Err1 = right_speed_pid.Err2 = 0.0f;
}
/**
 * @brief 增量式 PID 计算函数
 * @param pid 指向结构体的指针
 * @param target 目标速度 (cm/s)
 * @param actual 当前实际速度 (cm/s)
 * @return float 累计输出值 (PWM)
 */
float speed_pid_calc(Speed_PID_t *pid, float target, float actual)
{
    pid->Target = target;
    pid->Actual = actual;

    // 1. 更新误差记录
    pid->Err2 = pid->Err1;
    pid->Err1 = pid->Err0;
    pid->Err0 = pid->Target - pid->Actual;

    // 2. 增量式 PID 公式: 
    // ΔOut = Kp*(e0 - e1) + Ki*e0 + Kd*(e0 + e2 - 2*e1)
    float delta_out = pid->Kp * (pid->Err0 - pid->Err1) +
                      pid->Ki * (pid->Err0) +
                      pid->Kd * (pid->Err0 + pid->Err2 - 2.0f * pid->Err1);

    // 3. 累加输出
    pid->Out += delta_out;

    // 4. 输出限幅
    if (pid->Out > pid->Out_Limit)  pid->Out = pid->Out_Limit;
    if (pid->Out < -pid->Out_Limit) pid->Out = -pid->Out_Limit;

    return pid->Out;
}

/**
 * @brief 速度控制顶层更新函数
 * @details 包含获取编码器、PID计算、电机输出。建议在 PIT 中断中调用。
 */
void control_speed_update(float left_target, float right_target)
{
    // 1. 获取编码器原始计数值并清零
    int32_t left_raw = encoder_get_count(DIR_ENCODER_LEFT);
    encoder_clear_count(DIR_ENCODER_LEFT);
    
    int32_t right_raw = -encoder_get_count(DIR_ENCODER_RIGHT); // 注意右轮方向
    encoder_clear_count(DIR_ENCODER_RIGHT);

    // 2. 转换为物理速度 (单位: cm/s)
    float left_actual  = (float)left_raw  * PULSE_TO_CMS;
    float right_actual = (float)right_raw * PULSE_TO_CMS;

    // 3. 计算 PID 输出
    float out_l = speed_pid_calc(&left_speed_pid,  left_target,  left_actual);
    float out_r = speed_pid_calc(&right_speed_pid, right_target, right_actual);

    // 4. 电机执行驱动
    DRV7801_motion((int16_t)out_l, (int16_t)out_r);
}