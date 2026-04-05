/*
 * icm45686.h
 *
 *  Created on: 2025年12月31日
 *      Author: Lenovo
 */

#ifndef CODE_ICM45686_H_
#define CODE_ICM45686_H_

#include "zf_Common_Typedef.h"
#include "zf_Driver_Spi.h"
#include "zf_Driver_Gpio.h"
/***************************************************************
  *  @brief     SPI总线及引脚配置
 **************************************************************/
#define ICM_SPI         SPI_2                 // 使用 SPI_0
#define ICM_SPI_SCL     SPI2_CLK_P15_2       // SPI 时钟引脚 SCL/SCK
#define ICM_SPI_SDA     SPI2_MOSI_P15_1       // SPI 数据输出引脚 MOSI/SDA
#define ICM_SPI_SDO     SPI2_MISO_P15_0       // SPI 数据输入引脚 MISO/SDO
#define ICM_SPI_CS_PIN  P15_3                 // SPI 片选引脚 CS/NSS
#define ICM_SPI_MODE    SPI_MODE0             // SPI 模式 (CPOL=0, CPHA=0)
#define ICM_SPI_BAUD    10000000              // SPI 波特率

#define ICM_SET_SPI_CS(x) (x? (gpio_high(ICM_SPI_CS_PIN)): (gpio_low(ICM_SPI_CS_PIN)))
/***************************************************************
  *  @brief     三轴浮点数据结构体
 **************************************************************/
typedef struct
{
    float x;
    float y;
    float z;
} ImuAxis_t;  

/***************************************************************
  *  @brief     传感器数据集合
 **************************************************************/
typedef struct
{
    ImuAxis_t acc;                          // 加速度计数据 (单位: g, 重力加速度)
    ImuAxis_t gyro;                         // 陀螺仪数据 (单位: dps, 度/秒)
    ImuAxis_t mag;                          // 磁力计数据 (单位: uT, 微特斯拉)
    float temperature;                      // 温度数据
} ImuData_t;

/***************************************************************
  *  @brief     ICM45686加速度计量程 (Full-Scale)
 **************************************************************/
typedef enum
{
    ICM45686_AFS_32G = 0,        // ±32g
    ICM45686_AFS_16G,            // ±16g
    ICM45686_AFS_8G,             // ±8g
    ICM45686_AFS_4G,             // ±4g
    ICM45686_AFS_2G              // ±2g
} ICM45686Afs_t;

/***************************************************************
  *  @brief     ICM45686陀螺仪量程 (Full-Scale)
 **************************************************************/
typedef enum
{
    ICM45686_GFS_4000DPS = 0,    // ±4000dps
    ICM45686_GFS_2000DPS,        // ±2000dps
    ICM45686_GFS_1000DPS,        // ±1000dps
    ICM45686_GFS_500DPS,         // ±500dps
    ICM45686_GFS_250DPS,         // ±250dps
    ICM45686_GFS_125DPS,         // ±125dps
    ICM45686_GFS_62_5DPS,        // ±62.5dps
    ICM45686_GFS_31_25DPS,       // ±31.25dps
    ICM45686_GFS_15_625DPS       // ±15.625dps
} ICM45686Gfs_t;

/***************************************************************
  *  @brief     ICM45686加速度计输出数据速率 (Output Data Rate)
 **************************************************************/
typedef enum
{
    ICM45686_AODR_1_5625HZ = 0x0F,
    ICM45686_AODR_3_125HZ  = 0x0E,
    ICM45686_AODR_6_25HZ   = 0x0D,
    ICM45686_AODR_12_5HZ   = 0x0C,
    ICM45686_AODR_25HZ     = 0x0B,
    ICM45686_AODR_50HZ     = 0x0A,
    ICM45686_AODR_100HZ    = 0x09,
    ICM45686_AODR_200HZ    = 0x08,
    ICM45686_AODR_400HZ    = 0x07,
    ICM45686_AODR_800HZ    = 0x06,
    ICM45686_AODR_1600HZ   = 0x05,
    ICM45686_AODR_3200HZ   = 0x04,
    ICM45686_AODR_6400HZ   = 0x03
} ICM45686Aodr_t;

/***************************************************************
  *  @brief     ICM45686陀螺仪输出数据速率 (Output Data Rate)
 **************************************************************/
typedef enum
{
    ICM45686_GODR_1_5625HZ = 0x0F,
    ICM45686_GODR_3_125HZ  = 0x0E,
    ICM45686_GODR_6_25HZ   = 0x0D,
    ICM45686_GODR_12_5HZ   = 0x0C,
    ICM45686_GODR_25HZ     = 0x0B,
    ICM45686_GODR_50HZ     = 0x0A,
    ICM45686_GODR_100HZ    = 0x09,
    ICM45686_GODR_200HZ    = 0x08,
    ICM45686_GODR_400HZ    = 0x07,
    ICM45686_GODR_800HZ    = 0x06,
    ICM45686_GODR_1600HZ   = 0x05,
    ICM45686_GODR_3200HZ   = 0x04,
    ICM45686_GODR_6400HZ   = 0x03
} ICM45686Godr_t;

// ICM45686寄存器地址定义
#define ICM45686_WHO_AM_I           0x72      // 器件ID寄存器
#define ICM45686_REG_MISC2          0x7F      // 配置寄存器
#define ICM45686_PWR_MGMT0          0x10      // 电源管理寄存器0
#define ICM45686_TEMP_DATA1_UI      0x0C      // 温度数据寄存器起始地址
#define ICM45686_ACCEL_DATA_X1_UI   0x00      // 加速度数据寄存器起始地址
#define ICM45686_GYRO_DATA_X1_UI    0x06      // 陀螺仪数据寄存器起始地址
#define ICM45686_ACCEL_CONFIG0      0x1B      // 加速度计配置寄存器
#define ICM45686_GYRO_CONFIG0       0x1C      // 陀螺仪配置寄存器

extern ImuData_t gICM45686Data;

/***************************************************************
  *  @brief     向IMU传感器写入单个寄存器
  *  @param     reg     目标寄存器地址
  *  @param     dat     要写入的8位数据
  *  @Sample usage:     ICM_Write_Reg(0x20, 0x47);
 **************************************************************/
void ICM_Write_Reg(uint8_t reg, uint8_t dat);

/***************************************************************
  *  @brief     从IMU传感器连续读取多个寄存器
  *  @param     reg     起始寄存器地址
  *  @param     buffer  用于存储读取数据的缓冲区
  *  @param     length  要读取的字节数
  *  @Sample usage:     uint8_t data[6];
  *                     ICM_Read_Regs(0x28, data, 6);
 **************************************************************/
void ICM_Read_Regs(uint8_t reg, uint8_t *buffer, uint16_t length);

/***************************************************************
  *  @brief     通用IMU传感器初始化
  *  @param     None
  *  @Sample usage:     ICM_Init();
 **************************************************************/
void ICM_Init(void);



/***************************************************************
  *  @brief     初始化ICM45686传感器
  *  @param     None
  *  @Sample usage:     ICM45686_Init();
 **************************************************************/
void ICM45686_Init(void);

/***************************************************************
  *  @brief     从传感器读取最新的加速度计数据
  *  @param     None
  *  @Sample usage:     ICM45686_Read_Acc();
 **************************************************************/
void ICM45686_Read_Acc(void);

/***************************************************************
  *  @brief     从传感器读取最新的陀螺仪数据(弧度制数据)
  *  @param     None
  *  @Sample usage:     ICM45686_Read_Gyro();
 **************************************************************/
void ICM45686_Read_Gyro(void);

/***************************************************************
  *  @brief     从传感器读取温度数据
  *  @param     None
  *  @Sample usage:     ICM45686_Read_Temp();
 **************************************************************/
void ICM45686_Read_Temp(void);


#endif /* CODE_ICM45686_H_ */
