
#include "DRV7801_car_motion.h"

void DRV7801_car_motion_init(void){
  pwm_init (DRV7801_L_PWM ,10000,0);
  pwm_init (DRV7801_R_PWM ,10000,0);
  gpio_init(DRV7801_L_DIR,GPO,DRV7801_R_FD,GPO_PUSH_PULL);
  gpio_init(DRV7801_R_DIR,GPO,DRV7801_L_FD,GPO_PUSH_PULL);
}


void DRV7801_motor_brake(DRV7801_motor_set mode){
  switch(mode){
  case left_stop:
  pwm_set_duty(DRV7801_L_PWM,0);
  break;
  case right_stop:
  pwm_set_duty(DRV7801_R_PWM,0);
  break;
  case both_stop:
  pwm_set_duty(DRV7801_L_PWM,0);
  pwm_set_duty(DRV7801_R_PWM,0);
  break;}
}


void DRV7801_motor_launch(DRV7801_motor_set mode,uint16_t duty){
  switch(mode){
  case both_forward :
  pwm_set_duty(DRV7801_L_PWM,duty);
  pwm_set_duty(DRV7801_R_PWM,duty);
  gpio_set_level(DRV7801_L_DIR,DRV7801_L_FD);
  gpio_set_level(DRV7801_R_DIR,DRV7801_R_FD);
  break;
  
  case left_forward:
  gpio_set_level(DRV7801_L_DIR,DRV7801_L_FD);
  pwm_set_duty(DRV7801_L_PWM,duty);
  break;
  
  case left_reverse:
  gpio_set_level(DRV7801_L_DIR,DRV7801_L_BD);
  pwm_set_duty(DRV7801_L_PWM,duty);
  break;
  
  case right_forward:
  gpio_set_level(DRV7801_R_DIR,DRV7801_R_FD);
  pwm_set_duty(DRV7801_R_PWM,duty);
  break;
  
  case right_reverse:
  gpio_set_level(DRV7801_R_DIR,DRV7801_R_BD);
  pwm_set_duty(DRV7801_R_PWM,duty);
  break;}
}

void DRV7801_motion(int16_t left_speed,int16_t right_speed)
{
  if(left_speed>=0)
  {
    DRV7801_motor_launch(left_forward,left_speed);
  }
  else
  {
    DRV7801_motor_launch(left_reverse,-left_speed);
  }
  
  if(right_speed>=0)
  {
    DRV7801_motor_launch(right_forward,right_speed);
  }
  else
  {
    DRV7801_motor_launch(right_reverse,-right_speed);
  }
}