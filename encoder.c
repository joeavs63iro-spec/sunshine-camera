#include "encoder.h"

void DIR_encoder_init(void){
  encoder_dir_init (DIR_ENCODER_LEFT ,  DIR_ENCODER_LEFT_PULSE,    DIR_ENCODER_LEFT_DIR);       
  encoder_dir_init (DIR_ENCODER_RIGHT,  DIR_ENCODER_RIGHT_PULSE,   DIR_ENCODER_RIGHT_DIR);
}

//注意 左编码器正转是正  右编码器正转是负
void DIR_encoder_get_count(volatile int16* encoder_data_dir){
  encoder_data_dir[0] = encoder_get_count(DIR_ENCODER_LEFT);
  encoder_clear_count(DIR_ENCODER_LEFT);
  encoder_data_dir[1] = encoder_get_count(DIR_ENCODER_RIGHT);
  encoder_clear_count(DIR_ENCODER_RIGHT);
}
