#ifndef EASY_CAN_H_
#define EASY_CAN_H_

#include "zf_common_headfile.h"

// --- CAN 控制器核心外设资源 ---
#define CAN_HW              CY_CANFD0_0_TYPE           // 绑定的 CAN 控制器底层寄存器基地址 (CANFD1_通道0)
#define CAN_IRQ             canfd_0_interrupts0_0_IRQn // 绑定的 CAN 硬件中断向量号
#define CAN_PCLK            PCLK_CANFD0_CLOCK_CAN0     // 绑定的 CAN 外设时钟总线源

// --- CAN 发送引脚 (TX) 物理配置 ---
#define TX_PORT             GPIO_PRT8                // TX 所属的物理 GPIO 端口 (Port 14)
#define TX_PIN              0u                         // TX 所在的物理引脚号 (Pin 0)
#define TX_MUX              P8_0_CANFD0_TTCAN_TX0     // TX 多路复用(MUX)选择: 将 P14.0 内部道岔切至 CAN_TX0

// --- CAN 接收引脚 (RX) 物理配置 ---
#define RX_PORT             GPIO_PRT8                 // RX 所属的物理 GPIO 端口 (Port 14)
#define RX_PIN              1u                         // RX 所在的物理引脚号 (Pin 1)
#define RX_MUX              P8_1_CANFD0_TTCAN_RX0     // RX 多路复用(MUX)选择: 将 P14.1 内部道岔切至 CAN_RX0

// =========================================================
extern volatile uint8_t can_rx_flag;      
extern cy_stc_canfd_msg_t can_rx_msg;     
extern volatile uint8_t can_tx_done_flag; 

void easy_can_init(void);
uint8_t easy_CAN_Send_Msg(uint8_t *msg, uint8_t len, uint32_t Msg_id);

#endif /* EASY_CAN_H_ */