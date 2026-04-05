#include "ipcdata.h"
#include "zf_common_headfile.h" // 包含逐飞通用头文件，保证类型正确

/**
 * @brief 将智能车 X 和 Y 轴物理距离(mm)打包进一个 uint32
 * @details 高 16 位存放 X 轴距离，低 16 位存放 Y 轴距离。
 *          因为单位是毫米，int16_t最大能表示正负32米，对车完全足够。
 */
uint32_t pack_car_data_simple(float dist_x, float dist_y) 
{
    // float 强转 int16_t 会自动舍弃小数保留整数，且保留正负号 (范围 -32768 ~ +32767 mm)
    int16_t x_int = (int16_t)dist_x; 
    int16_t y_int = (int16_t)dist_y;
    
    // 高16位存X，低16位存Y
    return ((uint32_t)(uint16_t)x_int << 16) | (uint16_t)y_int;
}

// 解包函数：自动还原出带正负号的 float (接收端 核1 使用)
void unpack_car_data_simple(uint32_t packed, float* dist_x, float* dist_y) 
{
    // 强转 int16_t 会让 C 语言自动处理最高位的符号位，负数完美还原
    *dist_x = (float)((int16_t)(packed >> 16));
    *dist_y = (float)((int16_t)(packed & 0xFFFF));
}

// -------------------------------------------------------------
// 原有的画十字函数，保持不变
// -------------------------------------------------------------
void draw_cross(float fx, float fy, uint16 color) 
{
    int16 x = (int16)fx;
    int16 y = (int16)fy;
    if (x < 15 || x > 172 || y < 15 || y > 104) return; // 边缘预留，防止画线越界

    ips114_draw_line(x - 15, y, x + 15, y, color); // 横线
    ips114_draw_line(x, y - 15, x, y + 15, color); // 竖线
}