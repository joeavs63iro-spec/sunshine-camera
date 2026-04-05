#ifndef _control_speed_h_
#define _control_speed_h_
#include "zf_common_headfile.h"
#include "car_headfile.h"
// 变量声明（方便蓝牙调参访问）
extern float Kp_Vision_Dist;
extern float Max_Base_Speed;

// 函数声明
void pid_camera_init(void);
void Beacon_Follow_Logic(void);
#endif