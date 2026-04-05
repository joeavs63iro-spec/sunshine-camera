#include "icm42688.h"
#include "zf_common_headfile.h"
#include <stdio.h>
#include <math.h>

#define ICM42688_AUTO_CALIBRATE_ON_STARTUP

// 全局变量定义
ImuData_t_6 gICM42688Data;
icm42688_gyro_param_t gyro_offset_42688;
icm42688_math_param_t icm42688_data;
icm42688_quater_param_t q_info_42688 = {1, 0, 0, 0};
icm42688_euler_param_t euler_angle_42688;

static float gAccScale = 1.0f;
static float gGyroScale = 1.0f;

// 姿态解算相关宏定义与参数
#define DELTA_T     0.005f
#define M_PI        3.14159265f
#define LPF_ALPHA   0.3f    // 一阶低通滤波器的alpha值

static float param_kp = 3.0f;     // 加速度计收敛速率的比例增益
static float param_ki = 0.004f;   // 陀螺仪收敛速率的积分增益
static float i_error_x = 0, i_error_y = 0, i_error_z = 0; // 误差积分

// SPI协议读写操作宏定义
#if ICM42688_HARD_SPI
#define ICM42688_Write_Reg(reg, data)       spi_write_8bit_register(ICM42688_SPI, reg, data);
#define ICM42688_Read_Regs(reg, data, num)  spi_read_8bit_registers(ICM42688_SPI, reg | 0x80, data, num);
#endif

// 内部静态函数声明
static void Write_Data_ICM42688(unsigned char reg, unsigned char data);
static void Read_Datas_ICM42688(unsigned char reg, unsigned char *data, unsigned int num);
static float Sqrt_Fast(float x);
static void Get_Values_ICM_42688(void);
static void Update_AHRS_ICM_42688(float gx, float gy, float gz, float ax, float ay, float az);

/*------------------------- 底层驱动部分 -------------------------*/

static void Write_Data_ICM42688(unsigned char reg, unsigned char data) {
    ICM42688_CS(0);
    ICM42688_Write_Reg(reg, data);
    ICM42688_CS(1);
}

static void Read_Datas_ICM42688(unsigned char reg, unsigned char *data, unsigned int num) {
    ICM42688_CS(0);
    ICM42688_Read_Regs(reg, data, num);
    ICM42688_CS(1);
}

void Delay_Ms_ICM42688(unsigned int ms) {
    while (--ms) {
        unsigned int x = 2501;
        while (--x);
    }
}

void Set_LowpassFilter_Range_ICM42688(ICM42688Afs_t afs, ICM42688Aodr_t aodr, ICM42688Gfs_t gfs, ICM42688Godr_t godr) {
    Write_Data_ICM42688(ICM42688_ACCEL_CONFIG0, (afs << 5) | (aodr + 1));
    Write_Data_ICM42688(ICM42688_GYRO_CONFIG0, (gfs << 5) | (godr + 1));

    switch(afs) {
        case ICM42688_AFS_2G:  gAccScale = 2 / 32768.0f; break;
        case ICM42688_AFS_4G:  gAccScale = 4 / 32768.0f; break;
        case ICM42688_AFS_8G:  gAccScale = 8 / 32768.0f; break;
        case ICM42688_AFS_16G: gAccScale = 16 / 32768.0f; break;
        default:               gAccScale = 1; break;
    }
    switch(gfs) {
        case ICM42688_GFS_15_625DPS: gGyroScale = 15.625f / 32768.0f; break;
        case ICM42688_GFS_31_25DPS:  gGyroScale = 31.25f / 32768.0f; break;
        case ICM42688_GFS_62_5DPS:   gGyroScale = 62.5f / 32768.0f; break;
        case ICM42688_GFS_125DPS:    gGyroScale = 125.0f / 32768.0f; break;
        case ICM42688_GFS_250DPS:    gGyroScale = 250.0f / 32768.0f; break;
        case ICM42688_GFS_500DPS:    gGyroScale = 500.0f / 32768.0f; break;
        case ICM42688_GFS_1000DPS:   gGyroScale = 1000.0f / 32768.0f; break;
        case ICM42688_GFS_2000DPS:   gGyroScale = 2000.0f / 32768.0f; break;
        default:                     gGyroScale = 1; break;
    }
}

void Init_ICM42688(void) {
    // SPI+GPIO初始化
    spi_init(ICM42688_SPI, SPI_MODE0, ICM42688_SPI_SPEED, ICM42688_SPC_PIN, ICM42688_SDI_PIN, ICM42688_SDO_PIN, SPI_CS_NULL);
    gpio_init(ICM42688_CS_PIN, GPO, GPIO_HIGH, GPO_PUSH_PULL);
    
    // 读取陀螺仪型号验证
    unsigned char model = 0xff;
    while(1) {
        unsigned char time = 50;
        Read_Datas_ICM42688(ICM42688_WHO_AM_I, &model, 1);
        if(model == 0x47) break;
        else {
            Delay_Ms_ICM42688(10);
            time--;
            if(time <= 0) while(1); // 初始化失败，卡死
        }
    }
    
    Write_Data_ICM42688(ICM42688_PWR_MGMT0, 0x00);      // 复位设备
    Delay_Ms_ICM42688(10);
    Set_LowpassFilter_Range_ICM42688(ICM42688_AFS_16G, ICM42688_AODR_32000HZ, ICM42688_GFS_2000DPS, ICM42688_GODR_32000HZ);
    Write_Data_ICM42688(ICM42688_PWR_MGMT0, 0x0f);      // 设置为低噪声模式
    Delay_Ms_ICM42688(10);

    #ifdef ICM42688_AUTO_CALIBRATE_ON_STARTUP
    system_delay_ms(50);
    Init_Gyro_Offset_42688();
    #endif
    system_delay_ms(10);
}

void Get_Acc_ICM42688(void) {
    uint8_t data[6];
    int16_t rawX, rawY, rawZ;
    Read_Datas_ICM42688(ICM42688_ACCEL_DATA_X1, data, 6);
    rawX = ((int16_t)data[0] << 8) | data[1];
    rawY = ((int16_t)data[2] << 8) | data[3];
    rawZ = ((int16_t)data[4] << 8) | data[5];
    gICM42688Data.acc.x = (float)rawX * gAccScale;
    gICM42688Data.acc.y = (float)rawY * gAccScale;
    gICM42688Data.acc.z = (float)rawZ * gAccScale;
}

void Get_Gyro_ICM42688(void) {
    uint8_t data[6];
    int16_t rawX, rawY, rawZ;
    Read_Datas_ICM42688(ICM42688_GYRO_DATA_X1, data, 6);
    rawX = ((int16_t)data[0] << 8) | data[1];
    rawY = ((int16_t)data[2] << 8) | data[3];
    rawZ = ((int16_t)data[4] << 8) | data[5];
    gICM42688Data.gyro.x = (float)rawX * gGyroScale;
    gICM42688Data.gyro.y = (float)rawY * gGyroScale;
    gICM42688Data.gyro.z = (float)rawZ * gGyroScale;
        
    // 死区消除微小漂移
    if(gICM42688Data.gyro.x < 3.0f && gICM42688Data.gyro.x > -3.0f) gICM42688Data.gyro.x = 0;
    if(gICM42688Data.gyro.y < 3.0f && gICM42688Data.gyro.y > -3.0f) gICM42688Data.gyro.y = 0;
    if(gICM42688Data.gyro.z < 3.0f && gICM42688Data.gyro.z > -3.0f) gICM42688Data.gyro.z = 0;
}

/*------------------------- 姿态解算(AHRS)部分 -------------------------*/

void Init_Gyro_Offset_42688(void) {
    unsigned int i;
    gyro_offset_42688.x_data = 0;
    gyro_offset_42688.y_data = 0;
    gyro_offset_42688.z_data = 0;

    for (i = 0; i < 200;) {
        Get_Gyro_ICM42688();
        // 舍弃离谱数据
        if(gICM42688Data.gyro.x > -10 && gICM42688Data.gyro.x < 10
           && gICM42688Data.gyro.y > -10 && gICM42688Data.gyro.y < 10
           && gICM42688Data.gyro.z > -10 && gICM42688Data.gyro.z < 10) {
            ++i;
            gyro_offset_42688.x_data += gICM42688Data.gyro.x;
            gyro_offset_42688.y_data += gICM42688Data.gyro.y;
            gyro_offset_42688.z_data += gICM42688Data.gyro.z;
        }
        system_delay_ms(1);
    }

    gyro_offset_42688.x_data /= 200;
    gyro_offset_42688.y_data /= 200;
    gyro_offset_42688.z_data /= 200;
    
    q_info_42688.q0 = 1.0f; q_info_42688.q1 = 0.0f; q_info_42688.q2 = 0.0f; q_info_42688.q3 = 0.0f;
    i_error_x = 0; i_error_y = 0; i_error_z = 0;
}

static float Sqrt_Fast(float x) {
    float halfx = 0.5f * x;
    float y = x;
    long i = *(long *) &y;
    i = 0x5f3759df - (i >> 1);
    y = *(float *) &i;
    y = y * (1.5f - (halfx * y * y));
    return y;
}

static void Get_Values_ICM_42688(void) {
    static double last_acc[3] = {0, 0, 0};
    
    // 一阶低通滤波
    icm42688_data.acc_x = (((float) gICM42688Data.acc.x) * LPF_ALPHA) + last_acc[0] * (1 - LPF_ALPHA);
    icm42688_data.acc_y = (((float) gICM42688Data.acc.y) * LPF_ALPHA) + last_acc[1] * (1 - LPF_ALPHA);
    icm42688_data.acc_z = (((float) gICM42688Data.acc.z) * LPF_ALPHA) + last_acc[2] * (1 - LPF_ALPHA);

    last_acc[0] = icm42688_data.acc_x;
    last_acc[1] = icm42688_data.acc_y;
    last_acc[2] = icm42688_data.acc_z;

    // 陀螺仪角度转弧度
    icm42688_data.gyro_x = ((float)gICM42688Data.gyro.x - gyro_offset_42688.x_data) * M_PI / 180.0f;
    icm42688_data.gyro_y = ((float)gICM42688Data.gyro.y - gyro_offset_42688.y_data) * M_PI / 180.0f;
    icm42688_data.gyro_z = ((float)gICM42688Data.gyro.z - gyro_offset_42688.z_data) * M_PI / 180.0f;
}

static void Update_AHRS_ICM_42688(float gx, float gy, float gz, float ax, float ay, float az) {
    float half_t = 0.5f * DELTA_T;
    float vx, vy, vz;
    float ex, ey, ez;
    float q0 = q_info_42688.q0;
    float q1 = q_info_42688.q1;
    float q2 = q_info_42688.q2;
    float q3 = q_info_42688.q3;

    float q0q0 = q0 * q0; float q0q1 = q0 * q1; float q0q2 = q0 * q2;
    float q1q1 = q1 * q1; float q1q3 = q1 * q3;
    float q2q2 = q2 * q2; float q2q3 = q2 * q3;
    float q3q3 = q3 * q3;

    float norm = Sqrt_Fast(ax * ax + ay * ay + az * az);
    if(norm == 0.0f) return; // 避免除0
    ax = ax * norm; ay = ay * norm; az = az * norm;

    vx = 2 * (q1q3 - q0q2);
    vy = 2 * (q0q1 + q2q3);
    vz = q0q0 - q1q1 - q2q2 + q3q3;

    ex = ay * vz - az * vy;
    ey = az * vx - ax * vz;
    ez = ax * vy - ay * vx;

    i_error_x += half_t * ex;
    i_error_y += half_t * ey;
    i_error_z += half_t * ez;
    
    gx = gx + param_kp * ex + param_ki * i_error_x;
    gy = gy + param_kp * ey + param_ki * i_error_y;
    gz = gz + param_kp * ez + param_ki * i_error_z;

    q0 = q0 + (-q1 * gx - q2 * gy - q3 * gz) * half_t;
    q1 = q1 + ( q0 * gx + q2 * gz - q3 * gy) * half_t;
    q2 = q2 + ( q0 * gy - q1 * gz + q3 * gx) * half_t;
    q3 = q3 + ( q0 * gz + q1 * gy - q2 * gx) * half_t;

    norm = Sqrt_Fast(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
    q_info_42688.q0 = q0 * norm;
    q_info_42688.q1 = q1 * norm;
    q_info_42688.q2 = q2 * norm;
    q_info_42688.q3 = q3 * norm;
}

void Get_Angles_ICM_42688(void) {
    float q0, q1, q2, q3;

    Get_Acc_ICM42688();
    Get_Gyro_ICM42688();
    Get_Values_ICM_42688();
    Update_AHRS_ICM_42688(icm42688_data.gyro_x, icm42688_data.gyro_y, icm42688_data.gyro_z, 
                          icm42688_data.acc_x, icm42688_data.acc_y, icm42688_data.acc_z);

    q0 = q_info_42688.q0;
    q1 = q_info_42688.q1;
    q2 = q_info_42688.q2;
    q3 = q_info_42688.q3;
    
    // 四元数计算欧拉角
    euler_angle_42688.pitch = asin(-2 * q1 * q3 + 2 * q0 * q2) * 180 / M_PI - 180;
    euler_angle_42688.roll = atan2(2 * q2 * q3 + 2 * q0 * q1, -2 * q1 * q1 - 2 * q2 * q2 + 1) * 180 / M_PI;
    euler_angle_42688.yaw = atan2(2 * q1 * q2 + 2 * q0 * q3, -2 * q2 * q2 - 2 * q3 * q3 + 1) * 180 / M_PI;

    // 姿态限制
    if (euler_angle_42688.pitch > 0) {
        euler_angle_42688.pitch = 180 - euler_angle_42688.pitch;
    } else if (euler_angle_42688.pitch < 0) {
        euler_angle_42688.pitch = -(180 + euler_angle_42688.pitch);
    }
    
    if (euler_angle_42688.yaw > 360) {
        euler_angle_42688.yaw -= 360;
    } else if (euler_angle_42688.yaw < 0) {
        euler_angle_42688.yaw += 360;
    }
    
    if (euler_angle_42688.yaw > 180) {
        euler_angle_42688.yaw -= 360;
    }
}