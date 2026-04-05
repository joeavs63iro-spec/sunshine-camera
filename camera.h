#ifndef _camera_h_
#define _camera_h_

#include "zf_common_headfile.h"

#define MAX_LABELS  256  
#define MAX_TARGETS 10                  

//相机内参
#define FX 72.2495f
#define FY 72.4339f
#define CX 96.3522f
#define CY 61.3585f

// 畸变参数
#define K1 -0.2569
#define K2 0.0502

// 图像中心
#define IMG_W 188
#define IMG_H 120
#define IMG_CENTER_U 94.0f
#define IMG_CENTER_V 60.0f
#define CAR_AXLE_DIST      250.0f  // 轴距 250mm
#define CAR_AXLE_ERR       50.0f   // 误差 20mm
#define BEACON_MIN_AREA    20      // 判定为信标的最小面积阈值
#define CENTER_SAFE_RADIUS 60      //中心安全区
// 单应性矩阵 
#define H00 -0.05706857f
#define H01 -0.00000000f
#define H02 5.46716900f
#define H10 -0.00000000f
#define H11 -0.04004628f
#define H12 5.40828958f
#define H20 0.00000000f
#define H21 0.08899172f
#define H22 1.00000000f

typedef struct {
    uint32 area;
    uint32 sum_x;
    uint32 sum_y;
} LabelStats;

typedef struct {
    int x;
    int y;
    float nx_corr;
    float ny_corr;
    float world_x;
    float world_y;
    uint32 area;
} Target_t;

// 函数声明
void UF_Init(uint8 n);
uint8 UF_Find(uint8 i);
void UF_Union(uint8 i, uint8 j);
void homography_transform(float x_img, float y_img, float* x_world, float* y_world);
void Apply_Inverse_Perspective(Target_t* list, uint8 count, float* xb, float* yb);
void Sort_Targets_By_Area(Target_t* list, uint8 count);
uint8 Calculate_Centroids(uint8 num_flag, Target_t* out_list);
uint8 Connected_Components_Labeling(uint8 img[MT9V03X_H][MT9V03X_W]);
void Apply_Distortion_Correction(Target_t* list, uint8 count,float height_mm);
uint8 Identify_TriangleCar_And_Beacons(Target_t* list, uint8* count) ;
#endif