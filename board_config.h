/*********************************************************************************************************************
* 麦克纳姆轮底盘控制与全局定位系统
* Copyright (c) 2024
*
* 文件名称          board_config.h
* 功能描述          全车硬件引脚映射与物理参数配置字典
* 备注说明          完全适配逐飞 CYT4BB7 官方枚举规范。修改任何引脚或物理尺寸，仅需在此文件操作，实现与业务层解耦。
* 版本信息          V1.4 (终极注释版)
********************************************************************************************************************/
#ifndef _BOARD_CONFIG_H
#define _BOARD_CONFIG_H

#include "zf_common_headfile.h" 

//===================================================================================================================
// 1. 控制系统周期与机械物理参数定义
//===================================================================================================================
#define CTRL_PERIOD_MS       20.0f              // 运动控制与 PID 闭环任务的调度周期 (单位: ms)
#define IMU_PERIOD_MS        5.0f               // 姿态陀螺仪高频积分任务的调度周期 (单位: ms)

#define WHEEL_RADIUS         0.065f             // 麦克纳姆轮的物理半径 (单位: m，30mm = 0.03m)
#define GMR_PPR              500.0f             // GMR 编码器单圈产生的物理脉冲线数
#define REDUCTION_RATIO      30.0f              // 驱动电机自带的减速箱减速比 (例如 1:30)

/*
 * @brief 核心转换系数：脉冲数转物理位移 (m/pulse)
 * @note  车轮转动一圈的物理距离 = 2 * PI * R
 * 车轮转动一圈，编码器产生的总脉冲数 = GMR_PPR * REDUCTION_RATIO
 * 所以：1个脉冲代表的物理距离 = (2 * PI * R) / (GMR_PPR * REDUCTION_RATIO)
 */
#define PULSE_TO_METER       ((2.0f * 3.14159265f * WHEEL_RADIUS) / (GMR_PPR * REDUCTION_RATIO))

//===================================================================================================================
// 2. 调试与通信接口定义 (无线串口)
//===================================================================================================================
#define DEBUG_UART_IDX       UART_0             // 调试终端 UART 外设模块编号
//#define DEBUG_UART_TX_PIN    UART0_TX_P4_1      // 调试终端 TX 引脚 (复用枚举)
//#define DEBUG_UART_RX_PIN    UART0_RX_P4_0      // 调试终端 RX 引脚 (复用枚举)
#define DEBUG_UART_BAUD      115200             // 调试终端通信波特率

//===================================================================================================================
// 3. 惯性测量单元 (IMU) SPI 接口定义
//===================================================================================================================
#define ICM_SPI              SPI_2              // IMU SPI 硬件外设模块编号
#define ICM_SPI_SCL          SPI2_CLK_P15_2     // IMU SPI 时钟引脚 SCL
#define ICM_SPI_SDA          SPI2_MOSI_P15_1    // IMU SPI 数据输出 MOSI
#define ICM_SPI_SDO          SPI2_MISO_P15_0    // IMU SPI 数据输入 MISO
#define ICM_SPI_CS_PIN       P15_3              // IMU SPI 软件片选引脚 CS (使用普通 GPIO 控制)
#define ICM_SPI_MODE         SPI_MODE0          // IMU SPI 工作模式0 (CPOL=0, CPHA=0)
#define ICM_SPI_BAUD         10000000           // IMU SPI 通信速率 (10MHz)

//===================================================================================================================
// 4. 电机驱动器 PWM 接口定义 (严格匹配逐飞 zf_driver_pwm.h 的官方枚举)
//===================================================================================================================
// 左前轮 (LF - Left Front) 
#define LF_PWM_PIN           TCPWM_CH09_P05_0   // LF 轮 PWM 输出引脚通道枚举
#define LF_DIR_IN1           P09_0              // LF 轮方向控制引脚 1 (普通 GPIO)
#define LF_DIR_IN2           P09_1              // LF 轮方向控制引脚 2 (普通 GPIO)

// 左后轮 (LB - Left Back) 
#define LB_PWM_PIN           TCPWM_CH10_P05_1   // LB 轮 PWM 输出引脚通道枚举
#define LB_DIR_IN1           P07_3              // LB 轮方向控制引脚 1
#define LB_DIR_IN2           P07_2              // LB 轮方向控制引脚 2

// 右前轮 (RF - Right Front) 
#define RF_PWM_PIN           TCPWM_CH11_P05_2   // RF 轮 PWM 输出引脚通道枚举
#define RF_DIR_IN1           P19_0              // RF 轮方向控制引脚 1
#define RF_DIR_IN2           P19_1              // RF 轮方向控制引脚 2

// 右后轮 (RB - Right Back) 
#define RB_PWM_PIN           TCPWM_CH12_P05_3   // RB 轮 PWM 输出引脚通道枚举
#define RB_DIR_IN1           P10_3              // RB 轮方向控制引脚 1
#define RB_DIR_IN2           P10_2              // RB 轮方向控制引脚 2

//===================================================================================================================
// 5. 正交解码器 Encoder 接口定义 (严格匹配逐飞 zf_driver_encoder.h 的官方枚举)
//===================================================================================================================
// 左后轮编码器
#define ENC_LB_IDX           TC_CH07_ENCODER             // 定时器模块编号 (TIM_7)
#define ENC_LB_CH1           TC_CH07_ENCODER_CH1_P07_6   // A 相正交输入引脚枚举
#define ENC_LB_CH2           TC_CH07_ENCODER_CH2_P07_7   // B 相正交输入引脚枚举

// 右后轮编码器
#define ENC_RB_IDX           TC_CH20_ENCODER             // 定时器模块编号 (TIM_20)
#define ENC_RB_CH1           TC_CH20_ENCODER_CH1_P08_1   
#define ENC_RB_CH2           TC_CH20_ENCODER_CH2_P08_2   

// 右前轮编码器
#define ENC_RF_IDX           TC_CH27_ENCODER             // 定时器模块编号 (TIM_27)
#define ENC_RF_CH1           TC_CH27_ENCODER_CH1_P19_2   
#define ENC_RF_CH2           TC_CH27_ENCODER_CH2_P19_3   

// 左前轮编码器
#define ENC_LF_IDX           TC_CH58_ENCODER             // 定时器模块编号 (TIM_58)
#define ENC_LF_CH1           TC_CH58_ENCODER_CH1_P17_3   
#define ENC_LF_CH2           TC_CH58_ENCODER_CH2_P17_4   

#endif /* _BOARD_CONFIG_H */