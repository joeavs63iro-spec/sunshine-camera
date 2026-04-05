#ifndef _CONTROL_SPEED_H_
#define _CONTROL_SPEED_H_

#include "zf_common_headfile.h"
#include "car_headfile.h"

#define PULSE_TO_CMS  1.14f 
typedef struct
{
    float Kp;
    float Ki;
    float Kd;

    float Target;           // 目标值
    float Actual;           // 实际值
    
    float Err0;             // 本次误差
    float Err1;             // 上次误差
    float Err2;             // 上上次误差

    float Out;              // 累计输出值
    float Out_Limit;        // 输出限幅
} Speed_PID_t;

/* 外部变量声明 */
extern Speed_PID_t left_speed_pid;
extern Speed_PID_t right_speed_pid;

/* 函数声明 */
void pid_speed_init(void);
float speed_pid_calc(Speed_PID_t *pid, float target, float actual);
void control_speed_update(float left_target, float right_target);

#endif