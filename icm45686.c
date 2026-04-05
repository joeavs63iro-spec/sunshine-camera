/*
 * icm45686.c
 *
 *  Created on: 2025年12月31日
 *      Author: Lenovo
 */

#include "zf_common_headfile.h"
#include "icm45686.h"
#include <stdio.h>

#define ICM45686_AUTO_CALIBRATE_ON_STARTUP

ImuData_t gICM45686Data;

static float gAccScale = 1.0f;
static float gGyroScale = 1.0f;

static int16_t gAccBiasX = 0, gAccBiasY = 0, gAccBiasZ = 0;
static int16_t gGyroBiasX = 0, gGyroBiasY = 0, gGyroBiasZ = 0;




/***************************************************************
  *  @brief     向IMU传感器写入单个寄存器
  *  @param     reg     目标寄存器地址
  *  @param     dat     要写入的8位数据
  *  @Sample usage:     ICM_Write_Reg(0x20, 0x47);
 **************************************************************/
void ICM_Write_Reg(uint8_t reg, uint8_t dat)
{
    uint8_t buffer[2];
    buffer[0] = reg;
    buffer[1] = dat;

    ICM_SET_SPI_CS(0);
    system_delay_us(2);
    spi_write_8bit_array(ICM_SPI, buffer, 2);
    ICM_SET_SPI_CS(1);
}

/***************************************************************
  *  @brief     从IMU传感器连续读取多个寄存器
  *  @param     reg     起始寄存器地址
  *  @param     buffer  用于存储读取数据的缓冲区
  *  @param     length  要读取的字节数
  *  @Sample usage:     uint8_t data[6];
  *                     ICM_Read_Regs(0x28, data, 6);
 **************************************************************/
void ICM_Read_Regs(uint8_t reg, uint8_t *buffer, uint16_t length)
{
    ICM_SET_SPI_CS(0);
    system_delay_us(2);

    spi_write_8bit(ICM_SPI, reg | 0x80);

    spi_read_8bit_array(ICM_SPI, buffer, length);

    ICM_SET_SPI_CS(1);
}

/***************************************************************
  *  @brief     通用IMU传感器初始化
  *  @param     None
  *  @Sample usage:     ICM_Init();
 **************************************************************/
void ICM_Init(void)
{
    spi_init(ICM_SPI, ICM_SPI_MODE, ICM_SPI_BAUD,ICM_SPI_SCL, ICM_SPI_SDA, ICM_SPI_SDO, SPI_CS_NULL);

    system_delay_ms(1);

    gpio_init(ICM_SPI_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);

    system_delay_ms(10);
}



/***************************************************************
  *  @brief     设置加速度计的量程和数据输出速率
  *  @param     afs     量程 (Full-Scale)
  *  @param     aodr    数据输出速率
  *  @Sample usage:     ICM45686_Set_Afs_Aodr(ICM45686_AFS_16G, ICM45686_AODR_800HZ);
 **************************************************************/
static void ICM45686_Set_Afs_Aodr(ICM45686Afs_t afs, ICM45686Aodr_t aodr)
{
    uint8_t config = ((afs & 0x07) << 4) | (aodr & 0x0F);
    ICM_Write_Reg(ICM45686_ACCEL_CONFIG0, config);

    switch (afs)
    {
    case ICM45686_AFS_32G:  gAccScale = 1.0f / 1024.0f; break;
    case ICM45686_AFS_16G:  gAccScale = 1.0f / 2048.0f; break;
    case ICM45686_AFS_8G:   gAccScale = 1.0f / 4096.0f; break;
    case ICM45686_AFS_4G:   gAccScale = 1.0f / 8192.0f; break;
    case ICM45686_AFS_2G:   gAccScale = 1.0f / 16384.0f; break;
    default:                gAccScale = 1.0f; break;
    }
}

/***************************************************************
  *  @brief     设置陀螺仪的量程和数据输出速率
  *  @param     gfs     量程 (Full-Scale)
  *  @param     godr    数据输出速率
  *  @Sample usage:     ICM45686_Set_Gfs_Godr(ICM45686_GFS_2000DPS, ICM45686_GODR_800HZ);
 **************************************************************/
static void ICM45686_Set_Gfs_Godr(ICM45686Gfs_t gfs, ICM45686Godr_t godr)
{
    uint8_t config = ((gfs & 0x0F) << 4) | (godr & 0x0F);
    ICM_Write_Reg(ICM45686_GYRO_CONFIG0, config);

    switch (gfs)
    {
    case ICM45686_GFS_4000DPS:  gGyroScale = 1.0f / 8.2f; break;
    case ICM45686_GFS_2000DPS:  gGyroScale = 1.0f / 16.4f; break;
    case ICM45686_GFS_1000DPS:  gGyroScale = 1.0f / 32.8f; break;
    case ICM45686_GFS_500DPS:   gGyroScale = 1.0f / 65.5f; break;
    case ICM45686_GFS_250DPS:   gGyroScale = 1.0f / 131.0f; break;
    case ICM45686_GFS_125DPS:   gGyroScale = 1.0f / 262.0f; break;
    case ICM45686_GFS_62_5DPS:  gGyroScale = 1.0f / 524.3f; break;
    case ICM45686_GFS_31_25DPS: gGyroScale = 1.0f / 1048.6f; break;
    case ICM45686_GFS_15_625DPS:gGyroScale = 1.0f / 2097.2f; break;
    default:                    gGyroScale = 1.0f; break;
    }
}

/***************************************************************
  *  @brief     对IMU进行零点偏置校准
  *  @param     None
  *  @Sample usage:     ICM45686_Calibrate_Bias();
 **************************************************************/
static void ICM45686_Calibrate_Bias(void)
{
    const int numSamples = 200;
    uint8_t dataBuffer[6];
    int16_t rawX, rawY, rawZ;
    int32_t accSumX = 0, accSumY = 0, accSumZ = 0;
    int32_t gyroSumX = 0, gyroSumY = 0, gyroSumZ = 0;
    int16_t accZGravityRaw;
    int i;

    for (i = 0; i < numSamples; i++)
    {
        ICM_Read_Regs(ICM45686_ACCEL_DATA_X1_UI, dataBuffer, 6);
        rawX = (int16_t)((dataBuffer[1] << 8) | dataBuffer[0]);
        rawY = (int16_t)((dataBuffer[3] << 8) | dataBuffer[2]);
        rawZ = (int16_t)((dataBuffer[5] << 8) | dataBuffer[4]);
        accSumX += rawX;
        accSumY += rawY;
        accSumZ += rawZ;

        ICM_Read_Regs(ICM45686_GYRO_DATA_X1_UI, dataBuffer, 6);
        rawX = (int16_t)((dataBuffer[1] << 8) | dataBuffer[0]);
        rawY = (int16_t)((dataBuffer[3] << 8) | dataBuffer[2]);
        rawZ = (int16_t)((dataBuffer[5] << 8) | dataBuffer[4]);
        gyroSumX += rawX;
        gyroSumY += rawY;
        gyroSumZ += rawZ;

        system_delay_ms(5);
    }

    gGyroBiasX = (int16_t)(gyroSumX / numSamples);
    gGyroBiasY = (int16_t)(gyroSumY / numSamples);
    gGyroBiasZ = (int16_t)(gyroSumZ / numSamples);

    gAccBiasX = (int16_t)(accSumX / numSamples);
    gAccBiasY = (int16_t)(accSumY / numSamples);

    if(gAccScale == (1.0f / 2048.0f)) {
        accZGravityRaw = 2048;
    } else {
        accZGravityRaw = 0;
    }
    gAccBiasZ = (int16_t)(accSumZ / numSamples) - accZGravityRaw;
}

/***************************************************************
  *  @brief     初始化ICM45686传感器
  *  @param     None
  *  @Sample usage:     ICM45686_Init();
 **************************************************************/
void ICM45686_Init(void)
{
    ICM_Init();

    ICM_Write_Reg(ICM45686_REG_MISC2, 0x02);
    system_delay_ms(10);

    ICM_Write_Reg(ICM45686_PWR_MGMT0, 0x0F);

    ICM45686_Set_Afs_Aodr(ICM45686_AFS_16G, ICM45686_AODR_800HZ);
    ICM45686_Set_Gfs_Godr(ICM45686_GFS_2000DPS, ICM45686_GODR_800HZ);
#ifdef ICM45686_AUTO_CALIBRATE_ON_STARTUP
    system_delay_ms(50);

    ICM45686_Calibrate_Bias();
#endif
    system_delay_ms(10);
}

/***************************************************************
  *  @brief     从传感器读取最新的加速度计数据
  *  @param     None
  *  @Sample usage:     ICM45686_Read_Acc();
 **************************************************************/
void ICM45686_Read_Acc(void)
{
    uint8_t dataBuffer[6];
    int16_t rawX, rawY, rawZ;
    ICM_Read_Regs(ICM45686_ACCEL_DATA_X1_UI, dataBuffer, 6);

     rawX = (int16_t)((dataBuffer[1] << 8) | dataBuffer[0]);
     rawY = (int16_t)((dataBuffer[3] << 8) | dataBuffer[2]);
     rawZ = (int16_t)((dataBuffer[5] << 8) | dataBuffer[4]);
    rawX -= gAccBiasX;
    rawY -= gAccBiasY;
    rawZ -= gAccBiasZ;
    gICM45686Data.acc.x = (float)rawX * gAccScale;
    gICM45686Data.acc.y = (float)rawY * gAccScale;
    gICM45686Data.acc.z = (float)rawZ * gAccScale;
}

/***************************************************************
  *  @brief     从传感器读取最新的陀螺仪数据
  *  @param     None
  *  @Sample usage:     ICM45686_Read_Gyro();
 **************************************************************/
void ICM45686_Read_Gyro(void)
{
    uint8_t dataBuffer[6];
    int16_t rawX, rawY, rawZ;
    ICM_Read_Regs(ICM45686_GYRO_DATA_X1_UI, dataBuffer, 6);

     rawX = (int16_t)((dataBuffer[1] << 8) | dataBuffer[0]);
     rawY = (int16_t)((dataBuffer[3] << 8) | dataBuffer[2]);
     rawZ = (int16_t)((dataBuffer[5] << 8) | dataBuffer[4]);
    rawX -= gGyroBiasX;
    rawY -= gGyroBiasY;
    rawZ -= gGyroBiasZ;
    gICM45686Data.gyro.x = (float)rawX * gGyroScale;
    gICM45686Data.gyro.y = (float)rawY * gGyroScale;
    gICM45686Data.gyro.z = (float)rawZ * gGyroScale;
}

/***************************************************************
  *  @brief     从传感器读取温度数据
  *  @param     None
  *  @Sample usage:     ICM45686_Read_Temp();
 **************************************************************/
void ICM45686_Read_Temp(void)
{
    uint8_t dataBuffer[2];
    int16_t rawTemp;
    ICM_Read_Regs(ICM45686_TEMP_DATA1_UI, dataBuffer, 2);

     rawTemp = (int16_t)((dataBuffer[1] << 8) | dataBuffer[0]);

    gICM45686Data.temperature = (float)rawTemp / 132.48f + 25.0f;
}