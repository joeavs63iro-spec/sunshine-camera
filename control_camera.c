#include "control_camera.h"
#include <math.h>
extern uint32 u32;
extern float base_v;
float data_out[4]; 
float last_base_v = 0;
// 参数定义
float Kp_Vision_Dist = 100.0f;
float Max_Base_Speed = 300.0f;
void pid_camera_init(void) {
  Kp_Vision_Dist = 100.0f;
  Max_Base_Speed = 200.0f;
}
void Beacon_Follow_Logic(void) 
{
  
  float distance=(float)rx_peri_dist.field.dist_x/1000.0f;
  float visual_error_deg=(float)rx_peri_dist.field.dist_y  / PI/ 1000.0f* 180.0f;
  uint8_t car_is_locked=(uint8_t)rx_peri_dist.field.dist_y;
  if(car_is_locked)
  {
    if (distance > 0.001f) 
    {
      // --- 角度逻辑：只在视觉刷新时更新一次 target ---
      if (distance > 0.30f) 
      {
        yaw_pid.target = euler_angle.yaw + visual_error_deg;
        // 远距离速度：线性衰减，但给一个保底速度（比如100），防止卡死
        float calc_v = distance * Kp_Vision_Dist;
        base_v = (calc_v < 100) ? 100 : calc_v; 
      }
      else // 靠近信标：进入“加速侧弯”模式
      {
        float side = (visual_error_deg > 0) ? 1.0f : -1.0f;
        yaw_pid.target = euler_angle.yaw + visual_error_deg + (side * 25.0f);
        base_v = 100; 
      }
      // 角度归一化
      if (yaw_pid.target > 180.0f)  yaw_pid.target -= 360.0f;
      if (yaw_pid.target < -180.0f) yaw_pid.target += 360.0f;
    }
  }
  else
  {
    base_v = 0;
    yaw_pid.target = euler_angle.yaw;
    
  }
}