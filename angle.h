#ifndef _ANGLE_H_
#define _ANGLE_H_
#include "zf_common_headfile.h"
#include "icm45686.h"
// 加速度计收敛速率的比例增益,用于调整加速度计数据的收敛速度
extern float param_kp;   
// 陀螺仪收敛速率的积分增益,用于调整陀螺仪数据的积分收敛效果
extern float param_ki;
//void Get_Values_ICM(void);
// 存储陀螺仪的零点偏移数据
typedef struct {
    float x_data;
    float y_data;
    float z_data;
} gyro_param_t;

// 存储陀螺仪和加速度计的数据
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm_param_t;

// 存储四元数参数,四元数用于表示旋转
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} quater_param_t;

// 存储欧拉角参数,欧拉角用于表示三维空间中的旋转
typedef struct {
		float pitch;
		float roll;
		float yaw;
} euler_param_t;

// 声明结构体
extern gyro_param_t gyro_offset;
extern euler_param_t euler_angle;
extern icm_param_t icm_data;
extern float lpf_alpha;

/**
*
* @brief    陀螺仪零飘初始化
* @param    void
* @return   void
* @notes    初始化陀螺仪后调用
* Example:  Init_Gyro_Offset();
*
**/
void Init_Gyro_Offset(void);

/**
*
* @brief    四元数转换成欧拉角
* @param    void
* @return   void
* @notes    中断中固定周期调用
* Example:  Get_Angles_ICM();
*
**/
void Get_Angles_ICM(void);

#endif
