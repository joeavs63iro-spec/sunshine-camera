#ifndef _encoder_h_
#define _encoder_h_

#include "zf_common_headfile.h"
#define DIR_ENCODER_LEFT                     (TC_CH58_ENCODER)                      // 编码器接口       
#define DIR_ENCODER_LEFT_PULSE               (TC_CH58_ENCODER_CH1_P17_3)            // PULSE 对应的引脚                      
#define DIR_ENCODER_LEFT_DIR                 (TC_CH58_ENCODER_CH2_P17_4)            // DIR 对应的引脚                        
                                                                                
#define DIR_ENCODER_RIGHT                     (TC_CH27_ENCODER)                      // 编码器接口   
#define DIR_ENCODER_RIGHT_PULSE               (TC_CH27_ENCODER_CH1_P19_2)            // PULSE 对应的引脚                  
#define DIR_ENCODER_RIGHT_DIR                 (TC_CH27_ENCODER_CH2_P19_3)            // DIR 对应的引脚  

void DIR_encoder_init(void);
void DIR_encoder_get_count(volatile int16* encoder_data_dir);

#endif 