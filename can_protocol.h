#ifndef CAN_PROTOCOL_H_
#define CAN_PROTOCOL_H_

#include "zf_common_headfile.h"

// ==========================================
//    节点与报文 ID 宏定义 
// ==========================================
#define CAN_ID_EMERGENCY    0x000 // 全局急停
#define CAN_ID_PERI_ATT     0x121 // 【200Hz】外设板 -> 姿态
#define CAN_ID_PERI_DIST    0x221 // 【50Hz】外设板 -> 距离
#define CAN_ID_FC_STATUS    0x311 // 【50Hz】飞控板 -> 状态
#define CAN_ID_CAR_STATUS   0x331 // 【50Hz】智能车 -> 状态

#pragma pack(1) //强制 1 字节对齐，防止解包错位
// ==========================================
//  联合体声明
// ==========================================
typedef union 
{
    uint8_t data_bytes[8]; 
    struct {
        int16_t  pitch;       // Byte 0-1: 俯仰角 (*100)
        int16_t  roll;        // Byte 2-3: 横滚角 (*100)
        int16_t  yaw;         // Byte 4-5: 偏航角 (*100)
        uint8_t  seq_cnt;     // Byte 6:   防死机序列号 (0~255)
        uint8_t  valid_flag;  // Byte 7:   数据有效性 (0=无效, 1=有效)
    } field;
} msg_peri_attitude_t;

typedef union 
{
    uint8_t data_bytes[8];
    struct {
        uint16_t dist_x;      // Byte 0-1: X轴距离 (mm)
        uint16_t dist_y;      // Byte 2-3: Y轴距离 (mm)
        uint16_t dist_z;      // Byte 4-5: Z轴距离 (mm)
        uint8_t  seq_cnt;     // Byte 6:   防死机序列号
        uint8_t  valid_flag;  // Byte 7:   数据有效性
    } field;
} msg_peri_distance_t;

typedef union 
{
    uint8_t data_bytes[8];
    struct {
        uint8_t  car_mode;    // Byte 0: 模式 (0=停车, 1=跟随)
        uint8_t  error_code;  // Byte 1: 故障码 (0=正常)
        int16_t  speed_mm_s;  // Byte 2-3: 真实速度 (mm/s)
        uint8_t  reserved1;   // Byte 4: 预留
        uint8_t  reserved2;   // Byte 5: 预留
        uint8_t  seq_cnt;     // Byte 6:   防死机序列号
        uint8_t  valid_flag;  // Byte 7:   状态有效性
    } field;
} msg_car_status_t;

#pragma pack() // 恢复默认对齐
// ==========================================
//  全局变量声明
// ==========================================
extern msg_peri_attitude_t  rx_peri_att;
extern msg_peri_distance_t  rx_peri_dist;
extern msg_car_status_t     rx_car_status;
// ==========================================
//  函数声明
// ==========================================
void CAN_Unpack(uint32_t msg_id, uint8_t *data_payload);

#endif /* CAN_PROTOCOL_H_ */