
#ifndef _DRV7801_car_motion_h_
#define _DRV7801_car_motion_h_


#include "zf_common_headfile.h"
#include "icm45686.h"
#include "angle.h"
#define DRV7801_L_FD                1
#define DRV7801_L_BD                0 
#define DRV7801_L_DIR               P09_0
#define DRV7801_L_PWM               TCPWM_CH25_P09_1
#define DRV7801_R_FD                0
#define DRV7801_R_BD                1
#define DRV7801_R_DIR               P05_0
#define DRV7801_R_PWM               TCPWM_CH10_P05_1
typedef enum{
    left_reverse  ,
    left_forward   ,
    left_stop      ,

    right_reverse  ,
    right_forward  ,
    right_stop     ,
  
    both_forward   ,
    both_stop      ,
}DRV7801_motor_set;

void DRV7801_car_motion_init(void);
void DRV7801_motor_brake(DRV7801_motor_set mode);
void DRV7801_motor_launch(DRV7801_motor_set mode,uint16_t duty);
void DRV7801_motion(int16_t left_speed,int16_t right_speed);

#endif