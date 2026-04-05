#include "can_protocol.h"

// ==========================================
//  全局变量
// ==========================================
msg_peri_attitude_t  rx_peri_att;
msg_peri_distance_t  rx_peri_dist;
msg_car_status_t     rx_car_status;

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      CAN 接收报文解析与分发
//  参数说明      msg_id          接收到的 CAN 报文 ID (参考 can_protocol.h 中的宏定义)
//  参数说明      data_payload    指向 8 字节数据载荷的指针
//  返回参数      void            无
//  使用示例      CAN_Parse_Msg(pstcCanFDmsg->idConfig.identifier, temp_payload);
//  备注信息      内部函数，通常在 CAN 接收中断回调中调用，利用联合体自动完成数据解包对齐
//-------------------------------------------------------------------------------------------------------------------
void CAN_Unpack(uint32_t msg_id, uint8_t *data_payload)
{
    switch(msg_id)
    {
        case CAN_ID_PERI_ATT:
            memcpy(rx_peri_att.data_bytes, data_payload, 8);
            break;
            
        case CAN_ID_PERI_DIST:
            memcpy(rx_peri_dist.data_bytes, data_payload, 8);
            break;

        case CAN_ID_CAR_STATUS:
            memcpy(rx_car_status.data_bytes, data_payload, 8);
            break;

        default:
            break;
    }
}