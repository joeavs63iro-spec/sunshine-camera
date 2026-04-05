#ifndef CODE_CONTROL_ANGLE_H_
#define CODE_CONTROL_ANGLE_H_

#include "zf_Common_headfile.h"
#include "DRV7801_car_motion.h"
#include "angle.h"
/*PID结构体定义*/
typedef struct {
    float kp;
    float ki;
    float kd;
    float target;        // 目标角度
    float current_error;  // 当前误差
    float last_error;     // 上次误差
    float integral;       // 积分累加
    float i_limit;        // 积分限幅（防止 5 秒崩掉的关键）
    float output_limit;   // 输出限幅（PWM 最大值）
} PID_Param_t;


/*变量声明*/
extern PID_Param_t yaw_pid;

extern float lpf_alpha;

/*函数声明*/


void pid_angle_init(void);
float pid_calc_yaw(PID_Param_t *pid, float measure) ;
void control_angle(void) ;

#endif