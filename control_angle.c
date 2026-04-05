#include "control_angle.h"
/*变量定义*/
PID_Param_t yaw_pid;

/*函数定义*/

/**
 * @brief PID参数初始化
 * @param void
 * @return void
 */
void pid_angle_init(void)
{
    yaw_pid.kp = 5.6f;
    yaw_pid.ki = 0.1f;
    yaw_pid.kd = 70.0f;
    yaw_pid.target = 0.0f;       
    yaw_pid.current_error = 0.0f;  
    yaw_pid.last_error = 0.0f;     
    yaw_pid.integral = 0.0f;    
    yaw_pid.i_limit = 50.0f;
    yaw_pid.output_limit = 2000.0f;

}

/**
 * @brief 角度环 PID 计算
 * @param pid 指向 PID 结构体的指针
 * @param measure 当前测量到的角度 (euler_angle.yaw)
 * @return float 电机输出值 (PWM)
 */
float pid_calc_yaw(PID_Param_t *pid, float measure) 
{
    float output;
  
    pid->current_error = pid->target - measure;

    // 角度回环处理：确保转动路径最短
    if (pid->current_error > 180.0f)  pid->current_error -= 360.0f;
    if (pid->current_error < -180.0f) pid->current_error += 360.0f;

    if (pid->current_error < 3.0f && pid->current_error > -3.0f) 
    {
        return 0.0f; 
    }

    // 积分计算（含抗饱和限幅）
    pid->integral += pid->current_error;
    if(pid->integral > pid->i_limit)  
    {
      pid->integral = pid->i_limit;
    }
    if(pid->integral < -pid->i_limit) 
    {
      pid->integral = -pid->i_limit;
    }

    
    output = (pid->kp * pid->current_error) +  (pid->ki * pid->integral) - (pid->kd * icm_data.gyro_z);

    pid->last_error = pid->current_error;

    //输出限幅
    if(output > pid->output_limit)  
    {
      output = pid->output_limit;
    }
    if(output < -pid->output_limit) 
    {
      output = -pid->output_limit;
    }

    return output;
}

void control_angle(void) 
{
    float motor_pwm;
  

    // 计算 PWM 输出
    motor_pwm = pid_calc_yaw(&yaw_pid, euler_angle.yaw);
    
    if(motor_pwm<0)
    {
      DRV7801_motor_launch(left_reverse,(int16_t)fabsf(motor_pwm)); 
      DRV7801_motor_launch(right_forward,(int16_t)fabsf(motor_pwm)); 
    }
    else
    {
      DRV7801_motor_launch(left_forward,(int16_t)fabsf(motor_pwm)); 
      DRV7801_motor_launch(right_reverse,(int16_t)fabsf(motor_pwm)); 
    }
}