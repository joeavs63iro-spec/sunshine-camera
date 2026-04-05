/**
 * @file    mecanum_chassis.h
 * @author  Gemini (Technical Consultant)
 * @brief   Mecanum Wheel Chassis Driver (Cascaded PID + ICM42688 Integration)
 */

#ifndef _MECANUM_CHASSIS_H_
#define _MECANUM_CHASSIS_H_

#include "zf_common_headfile.h" 
#include "icm42688.h" /* 引入你的 42688 角度库 */

/* =========================================================================================
 * [物理机械参数与极限配置区] 
 * ========================================================================================= */
#define MECANUM_TRACK_WIDTH_MM      (198.0f)    
#define MECANUM_WHEEL_BASE_MM       (171.9f)    
#define MECANUM_WHEEL_RADIUS_MM     (37.5f)     

#define ENCODER_RESOLUTION          (500.0f)    
#define MOTOR_GEAR_RATIO            (30.0f)     

#define MECANUM_KINEMATIC_K         ((MECANUM_TRACK_WIDTH_MM + MECANUM_WHEEL_BASE_MM) / 2.0f)

#define MAX_PWM_DUTY                (10000)     
#define MAX_WHEEL_SPEED             (470.0f)    

#define ABS(x) ((x) > 0 ? (x) : -(x))           

/* =========================================================================================
 * [硬件引脚映射区]
 * ========================================================================================= */
#define ENC_MODULE_FL               (TC_CH58_ENCODER)
#define ENC_PIN_FL_A                (TC_CH58_ENCODER_CH1_P17_3)
#define ENC_PIN_FL_B                (TC_CH58_ENCODER_CH2_P17_4)

#define ENC_MODULE_FR               (TC_CH27_ENCODER)
#define ENC_PIN_FR_A                (TC_CH27_ENCODER_CH1_P19_2)
#define ENC_PIN_FR_B                (TC_CH27_ENCODER_CH2_P19_3)

#define ENC_MODULE_RL               (TC_CH07_ENCODER)
#define ENC_PIN_RL_A                (TC_CH07_ENCODER_CH1_P07_6)
#define ENC_PIN_RL_B                (TC_CH07_ENCODER_CH2_P07_7)

#define ENC_MODULE_RR               (TC_CH20_ENCODER)
#define ENC_PIN_RR_A                (TC_CH20_ENCODER_CH1_P08_1)
#define ENC_PIN_RR_B                (TC_CH20_ENCODER_CH2_P08_2)

#define MOTOR_PWM_FL                (TCPWM_CH09_P05_0)
#define MOTOR_PWM_RL                (TCPWM_CH10_P05_1)
#define MOTOR_PWM_FR                (TCPWM_CH11_P05_2)
#define MOTOR_PWM_RR                (TCPWM_CH12_P05_3)

#define MOTOR_DIR_FL_1              (P09_0)
#define MOTOR_DIR_FL_2              (P09_1)

#define MOTOR_DIR_FR_1              (P19_0)
#define MOTOR_DIR_FR_2              (P19_1)

#define MOTOR_DIR_RL_1              (P07_2)
#define MOTOR_DIR_RL_2              (P07_3)

#define MOTOR_DIR_RR_1              (P10_2)
#define MOTOR_DIR_RR_2              (P10_3)

/* =========================================================================================
 * [核心数据结构定义] (串级控制架构)
 * ========================================================================================= */
/**
 * @brief 姿态外环 PID 控制器 (航向角锁定)
 */
typedef struct {
    float kp;                   
    float kd;                   
    
    float yaw_offset;           /* ?? 极其关键：开机初始航向角偏置，用于软件归零 */
    float target_yaw;           /* 目标航向角 (相对零点) */
    float current_yaw;          /* 当前航向角 (相对零点) */
    
    float error;                
    float last_error;           
    
    float compensate_wz;        
} Heading_PID_TypeDef;

/**
 * @brief 速度内环 PID 控制器
 */
typedef struct {
    float kp;                   
    float ki;                   
    float kd;                   
    float kf;                   
    
    float target_velocity;      
    float current_velocity;     
    
    float error;                
    float last_error;           
    float integral;             
    
    float output_pwm;           
} Mecanum_Motor_TypeDef;

/**
 * @brief 整个麦轮底盘的状态结构体
 */
typedef struct {
    float vx;                   
    float vy;                   
    float wz;                   
    
    Heading_PID_TypeDef     yaw_pid;    
    
    Mecanum_Motor_TypeDef   wheel_FL;   
    Mecanum_Motor_TypeDef   wheel_FR; 
    Mecanum_Motor_TypeDef   wheel_RL; 
    Mecanum_Motor_TypeDef   wheel_RR; 
} Mecanum_Chassis_TypeDef;

extern Mecanum_Chassis_TypeDef Chassis;
extern volatile float global_pulse_sum;
/* =========================================================================================
 * [外部调用接口 API]
 * ========================================================================================= */
void Mecanum_Chassis_Init(void);
void Mecanum_Chassis_Calibrate_Yaw_Zero(void); /* ?? 航向角软件归零函数 */
void Mecanum_Set_Velocity(float vx, float vy, float wz);
void Mecanum_Set_Velocity_With_IMU(float vx, float vy, float user_wz); 
void Mecanum_Control_Task(void);

#endif /* _MECANUM_CHASSIS_H_ */