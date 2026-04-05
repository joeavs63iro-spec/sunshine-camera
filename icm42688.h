#ifndef _icm42688_h_
#define _icm42688_h_

#include "zf_common_debug.h"
#include "zf_driver_delay.h"
#include "zf_driver_spi.h"
#include "zf_driver_gpio.h"
#include "zf_driver_soft_iic.h"
#include "board_config.h"

// 通信方式选择,0:软件SPI,1:硬件SPI
#define ICM42688_HARD_SPI       1

/***************************************************************
  * @brief     SPI总线及引脚配置
 **************************************************************/
#if ICM42688_HARD_SPI
#define ICM42688_SPI_SPEED                  (ICM_SPI_BAUD)                          // 硬件 SPI 速率
#define ICM42688_SPI                        (ICM_SPI)                                 // 硬件 SPI 号
#define ICM42688_SPC_PIN                    (ICM_SPI_SCL )                       // 硬件 SPI SCK 引脚
#define ICM42688_SDI_PIN                    (ICM_SPI_SDA )                      // 硬件 SPI MOSI 引脚
#define ICM42688_SDO_PIN                    (ICM_SPI_SDO )                      // 定义SPI_MISO引脚
#endif


// ICM42688,CS管脚对应TC264引脚P15_3 (此处按原代码定义)
#define ICM42688_CS_PIN         ICM_SPI_CS_PIN

// 对ICM42688的CS管脚进行高低电平操作
#define ICM42688_CS(x)          ((x) ? (gpio_high(ICM42688_CS_PIN)) : (gpio_low(ICM42688_CS_PIN)))

/***************************************************************
  * @brief     驱动与姿态解算所需的数据结构
 **************************************************************/
// 三轴浮点数据结构体
typedef struct {
    float x;
    float y;
    float z;
} ImuAxis_t_6;

// 传感器原始数据集合
typedef struct {
    ImuAxis_t_6 acc;                          // 加速度计数据 (单位: g, 重力加速度)
    ImuAxis_t_6 gyro;                         // 陀螺仪数据 (单位: dps, 度/秒)
    float temperature;                        // 温度数据
} ImuData_t_6;

// 存储陀螺仪的零点偏移数据
typedef struct {
    float x_data;
    float y_data;
    float z_data;
} icm42688_gyro_param_t;

// 存储滤波后用于解算的物理量数据
typedef struct {
    float gyro_x;
    float gyro_y;
    float gyro_z;
    float acc_x;
    float acc_y;
    float acc_z;
} icm42688_math_param_t;

// 存储四元数参数
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
} icm42688_quater_param_t;

// 存储欧拉角参数
typedef struct {
    float pitch;
    float roll;
    float yaw;
} icm42688_euler_param_t;

/***************************************************************
  * @brief     ICM42688 量程和速率枚举
 *************************************************************/
typedef enum {
    ICM42688_AFS_16G,            // ±16g
    ICM42688_AFS_8G,             // ±8g
    ICM42688_AFS_4G,             // ±4g
    ICM42688_AFS_2G              // ±2g
} ICM42688Afs_t;

typedef enum {
    ICM42688_GFS_2000DPS,
    ICM42688_GFS_1000DPS,
    ICM42688_GFS_500DPS,
    ICM42688_GFS_250DPS,
    ICM42688_GFS_125DPS,
    ICM42688_GFS_62_5DPS,
    ICM42688_GFS_31_25DPS,
    ICM42688_GFS_15_625DPS,
    NUM_ICM42688_GFS
} ICM42688Gfs_t;

typedef enum {
    ICM42688_AODR_32000HZ,
    ICM42688_AODR_16000HZ,
    ICM42688_AODR_8000HZ,
    ICM42688_AODR_4000HZ,
    ICM42688_AODR_2000HZ,
    ICM42688_AODR_1000HZ,
    ICM42688_AODR_200HZ,
    ICM42688_AODR_100HZ,
    ICM42688_AODR_50HZ,
    ICM42688_AODR_25HZ,
    ICM42688_AODR_12_5HZ,
    ICM42688_AODR_6_25HZ,
    ICM42688_AODR_3_125HZ,
    ICM42688_AODR_1_5625HZ,
    ICM42688_AODR_500HZ,
    NUM_ICM42688_AODR
} ICM42688Aodr_t;

typedef enum {
    ICM42688_GODR_32000HZ,
    ICM42688_GODR_16000HZ,
    ICM42688_GODR_8000HZ,
    ICM42688_GODR_4000HZ,
    ICM42688_GODR_2000HZ,
    ICM42688_GODR_1000HZ,
    ICM42688_GODR_200HZ,
    ICM42688_GODR_100HZ,
    ICM42688_GODR_50HZ,
    ICM42688_GODR_25HZ,
    ICM42688_GODR_12_5HZ,
    ICM42688_GODR_X0HZ,
    ICM42688_GODR_X1HZ,
    ICM42688_GODR_X2HZ,
    ICM42688_GODR_500HZ,
    NUM_ICM42688_GODR
} ICM42688Godr_t;

// ICM42688 Bank0内部地址(仅截取使用的核心地址，其余已省去以保持整洁，可按需补充)
#define ICM42688_WHO_AM_I                  0x75
#define ICM42688_PWR_MGMT0                 0x4E
#define ICM42688_GYRO_CONFIG0              0x4F
#define ICM42688_ACCEL_CONFIG0             0x50
#define ICM42688_ACCEL_DATA_X1             0x1F
#define ICM42688_GYRO_DATA_X1              0x25

// 外部全局变量声明
extern ImuData_t_6 gICM42688Data;
extern icm42688_euler_param_t euler_angle_42688;
extern icm42688_quater_param_t q_info_42688;
extern icm42688_gyro_param_t gyro_offset_42688;

/*****************************************************************
* @brief    ICM42688 外部接口函数声明
*****************************************************************/
void Init_ICM42688(void);
void Get_Acc_ICM42688(void);
void Get_Gyro_ICM42688(void);
void Get_Gyro_ICM42688_raw(void);
void Set_LowpassFilter_Range_ICM42688(ICM42688Afs_t afs, ICM42688Aodr_t aodr, ICM42688Gfs_t gfs, ICM42688Godr_t godr);
void Delay_Ms_ICM42688(unsigned int ms);

// 姿态解算接口
void Init_Gyro_Offset_42688(void);
void Get_Angles_ICM_42688(void);

#endif