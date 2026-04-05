#ifndef _ipcdata_h_
#define _ipcdata_h_
#include "zf_common_headfile.h"

uint32_t pack_car_data_simple(float dist_x, float dist_y) ;
void unpack_car_data_simple(uint32_t packed, float* dist_x, float* dist_y);
void draw_cross(float fx, float fy,uint16 color) ;
#endif
