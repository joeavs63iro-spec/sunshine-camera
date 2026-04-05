#include "easy_can.h"
#include "can_protocol.h" 

volatile uint8_t can_rx_flag = 0;
volatile uint8_t can_tx_done_flag = 0;
cy_stc_canfd_msg_t can_rx_msg;

static const cy_stc_id_filter_t stdIdFilter[] = 
{
    CANFD_CONFIG_STD_ID_FILTER_RANGE(0x000, 0x7FF, CY_CANFD_ID_FILTER_ELEMNT_CONFIG_STORE_RXFIFO0)
};

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      CAN 硬件接收中断回调
//  参数说明      bRxFifoMsg            是否为 FIFO 消息
//  参数说明      u8MsgBufOrRxFifoNum   邮箱号或 FIFO 编号
//  参数说明      pstcCanFDmsg          存放底层接收数据的结构体指针
//  返回参数      void                  无
//  使用示例      硬件中断自动调用，用户无需关心
//  备注信息      该函数负责将 32 位寄存器数据还原为 8 字节纯净数组，并传递给上层协议引擎解析
//-------------------------------------------------------------------------------------------------------------------
void CAN_RxMsgCallback(bool bRxFifoMsg, uint8_t u8MsgBufOrRxFifoNum, cy_stc_canfd_msg_t* pstcCanFDmsg)
{
    can_rx_msg = *pstcCanFDmsg; 
    can_rx_flag = 1; 

    // 提取 8 字节数据并修复 CYT4BB7 底层的 32位错位
    uint8_t temp_payload[8];
    for(int i=0; i<8; i++) {
        temp_payload[i] = (uint8_t)(pstcCanFDmsg->dataConfig.data[i/4] >> ((i%4)*8)); 
    }
    
    // 丢给上层解析引擎，立刻变回物理变量
    CAN_Unpack(pstcCanFDmsg->idConfig.identifier, temp_payload);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      CAN 硬件发送完成中断回调
//  参数说明      void                  无
//  返回参数      void                  无
//  使用示例      硬件中断自动调用，用户无需关心
//  备注信息      当一帧数据在物理总线上成功收到 ACK 应答后触发，用于置位发送完成标志
//-------------------------------------------------------------------------------------------------------------------
void CAN_TxMsgCallback(void)
{
    can_tx_done_flag = 1; 
}

// 内部底层中断入口
void CanfdInterruptHandler(void)
{
    Cy_CANFD_IrqHandler(CAN_HW);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      CAN 硬件底层初始化
//  参数说明      void            无
//  返回参数      void            无
//  使用示例      easy_can_init(); 
//  备注信息      必须在主函数开启全局中断 (interrupt_global_enable) 之后调用。
//                默认配置：500kbps 波特率，标准帧，开启 RX_FIFO_0 接收中断与 TX 邮箱0发送完成中断。
//-------------------------------------------------------------------------------------------------------------------
void easy_can_init(void)
{
    uint32_t targetFreq = 40000000ul; 
    uint32_t periFreq   = 80000000ul; 
    uint32_t dividerNum = periFreq / targetFreq; 
    
    Cy_SysClk_PeriphAssignDivider(CAN_PCLK, CY_SYSCLK_DIV_8_BIT, 0ul);
    Cy_SysClk_PeriphSetDivider(Cy_SysClk_GetClockGroup(CAN_PCLK), CY_SYSCLK_DIV_8_BIT, 0ul, (dividerNum - 1ul));
    Cy_SysClk_PeriphEnableDivider(Cy_SysClk_GetClockGroup(CAN_PCLK), CY_SYSCLK_DIV_8_BIT, 0ul);

    cy_stc_gpio_pin_config_t pin_cfg = {0};
    pin_cfg.driveMode = CY_GPIO_DM_STRONG_IN_OFF; 
    pin_cfg.hsiom     = TX_MUX;
    pin_cfg.outVal    = 1ul;
    Cy_GPIO_Pin_Init(TX_PORT, TX_PIN, &pin_cfg);
    
    pin_cfg.driveMode = CY_GPIO_DM_HIGHZ;         
    pin_cfg.hsiom     = RX_MUX;
    Cy_GPIO_Pin_Init(RX_PORT, RX_PIN, &pin_cfg);

    cy_stc_canfd_config_t canCfg = {
        .txCallback            = CAN_TxMsgCallback, 
        .rxCallback            = CAN_RxMsgCallback, 
        .rxFifoWithTopCallback = NULL,
        .statusCallback        = NULL,
        .errorCallback         = NULL,
        .canFDMode             = false, 
        .bitrate = {
            .prescaler     = 4u - 1u, 
            .timeSegment1  = 14u - 1u, 
            .timeSegment2  = 5u  - 1u, 
            .syncJumpWidth = 4u  - 1u, 
        },
        .tdcConfig = { false, 0u, 0u }, 
        .sidFilterConfig = {
            .numberOfSIDFilters = sizeof(stdIdFilter) / sizeof(stdIdFilter[0]),
            .sidFilter          = stdIdFilter,
        },
        .globalFilterConfig = {
            .nonMatchingFramesStandard  = CY_CANFD_REJECT_NON_MATCHING,
            .nonMatchingFramesExtended  = CY_CANFD_REJECT_NON_MATCHING,
            .rejectRemoteFramesStandard = true,
            .rejectRemoteFramesExtended = true,
        },
        .rxBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
        .rxFifo1DataSize  = CY_CANFD_BUFFER_DATA_SIZE_8,
        .rxFifo0DataSize  = CY_CANFD_BUFFER_DATA_SIZE_8,
        .txBufferDataSize = CY_CANFD_BUFFER_DATA_SIZE_8,
        .rxFifo0Config    = {
            .mode                   = CY_CANFD_FIFO_MODE_OVERWRITE,
            .watermark              = 0u,
            .numberOfFifoElements   = 8u, 
            .topPointerLogicEnabled = false,
        },
        .noOfRxBuffers  = 0u,
        .noOfTxBuffers  = 8u,
    };

    Cy_CANFD_DeInit(CAN_HW);
    Cy_CANFD_Init(CAN_HW, &canCfg);

    // ★ 强制开启硬件中断线路
    CAN_HW->M_TTCAN.unIR.u32Register = 0xFFFFFFFF; 
    CAN_HW->M_TTCAN.unIE.u32Register |= (1ul << 0); // RX FIFO 0
    CAN_HW->M_TTCAN.unTXBTIE.u32Register |= (1ul << 0); 
    CAN_HW->M_TTCAN.unIE.u32Register |= (1ul << 9); // TX Complete
    CAN_HW->M_TTCAN.unILE.u32Register |= (1ul << 0); 

    cy_stc_sysint_irq_t irq_cfg = {
        .sysIntSrc = CAN_IRQ, 
        .intIdx    = CPUIntIdx2_IRQn,
        .isEnabled = true,
    };
    Cy_SysInt_InitIRQ(&irq_cfg);
    Cy_SysInt_SetSystemIrqVector(irq_cfg.sysIntSrc, CanfdInterruptHandler);
    NVIC_SetPriority(irq_cfg.intIdx, 0ul);
    NVIC_ClearPendingIRQ(irq_cfg.intIdx);
    NVIC_EnableIRQ(irq_cfg.intIdx);
}

//-------------------------------------------------------------------------------------------------------------------
//  函数简介      CAN 发送标准数据帧 (底层驱动)
//  参数说明      msg             指向需要发送的数据数组指针 (通常是联合体的 data_bytes 数组)
//  参数说明      len             发送的数据长度 (范围: 1 - 8 字节)
//  参数说明      Msg_id          该帧报文的 11 位标准 ID
//  返回参数      uint8_t         0: 发送成功 (报文已成功放入邮箱)  1: 发送失败
//  使用示例      easy_CAN_Send_Msg(tx_msg.data_bytes, 8, CAN_ID_PERI_ATT);
//  备注信息      底层已修复 CYT4BB7 硬件 32 位寄存器的字节拼装错位 Bug，可放心传入单字节数组
//-------------------------------------------------------------------------------------------------------------------
uint8_t easy_CAN_Send_Msg(uint8_t *msg, uint8_t len, uint32_t Msg_id)
{
    cy_stc_canfd_msg_t txMsg = {0};
    if(len > 8) len = 8; 
    
    txMsg.canFDFormat               = false; 
    txMsg.idConfig.extended         = false; 
    txMsg.idConfig.identifier       = Msg_id;
    txMsg.dataConfig.dataLengthCode = len;

    txMsg.dataConfig.data[0] = 0;
    txMsg.dataConfig.data[1] = 0;

    // 数据拼装：修复 32位 数组移位发送 Bug
    for(uint8_t i = 0; i < len; i++) {
        txMsg.dataConfig.data[i/4] |= ((uint32_t)msg[i] << ((i % 4) * 8));
    }

    if (Cy_CANFD_UpdateAndTransmitMsgBuffer(CAN_HW, 0u, &txMsg) == CY_CANFD_SUCCESS) {
        return 0; 
    }
    return 1;     
}