/**
* @file camera.c
* @brief 相机图像处理核心算法库
* @details 包含连通域标记(CCL)、并查集、畸变矫正、单应性变换及目标排序等功能
*/

#include "camera.h"

// =====================================================================
// ========== 内部静态变量 (私有) ==========
// =====================================================================

/** 
* @brief 图像标签映射表，记录每个像素点所属的连通域标签 
*/
static uint8 copy_label[MT9V03X_H][MT9V03X_W];

/** 
* @brief 连通域统计信息数组，索引为标签号 
*/
static LabelStats stats[256]; 

/** 
* @brief 并查集父节点数组，用于合并相连的连通域 
*/
static uint8 parent[MAX_LABELS]; 

/** 
* @brief 引用外部定义的原始图像数组 
*/
extern uint8 mt9v03x_image[MT9V03X_H][MT9V03X_W];


// =====================================================================
// ========== 并查集 (Union-Find) 算法实现 ==========
// =====================================================================

/**
* @brief 初始化并查集
* @param n 需要初始化的标签最大数量
*/
void UF_Init(uint8 n) {
  for (int i = 0; i <= n; i++) parent[i] = i;
}

/**
* @brief 并查集查找函数（带路径压缩优化）
* @param i 要查找的标签
* @return 根节点的标签号
*/
uint8 UF_Find(uint8 i) {
  if (parent[i] == i) return i;
  // 递归查找并将路径上的节点全部指向根节点，大幅提升后续查找速度
  return parent[i] = UF_Find(parent[i]); 
}

/**
* @brief 并查集合并函数
* @param i 标签1
* @param j 标签2
*/
void UF_Union(uint8 i, uint8 j) {
  uint8 root_i = UF_Find(i);
  uint8 root_j = UF_Find(j);
  if (root_i != root_j) parent[root_i] = root_j; 
}


// =====================================================================
// ========== 几何变换算法 ==========
// =====================================================================

/**
* @brief 单应性变换（逆透视变换）
* @details 将图像像素坐标通过 3x3 矩阵转换为物理世界坐标
* @param x_img 输入：像素横坐标
* @param y_img 输入：像素纵坐标
* @param x_world 输出：世界坐标X（通常为地面实际距离）
* @param y_world 输出：世界坐标Y（通常为地面实际距离）
*/
void homography_transform(float x_img, float y_img, float* x_world, float* y_world) {
  // 计算齐次坐标分量 w
  float w = H20 * x_img + H21 * y_img + H22;
  // 透视除法得到实际坐标
  *x_world = (H00 * x_img + H01 * y_img + H02) / w;
  *y_world = (H10 * x_img + H11 * y_img + H12) / w;
}


// =====================================================================
// ========== 图像处理核心流程 ==========
// =====================================================================

/**
* @brief 连通域标记算法 (CCL)
* @details 使用两遍扫描法识别图像中的白色连通区域并统计像素分布
* @param img 输入的二值化图像数组
* @return uint8 返回初步识别出的标签总数
*/
uint8 Connected_Components_Labeling(uint8 img[MT9V03X_H][MT9V03X_W]) {
  uint8 num_flag = 0;
  UF_Init(255);
  memset(copy_label, 0, sizeof(copy_label));
  memset(stats, 0, sizeof(stats));
  
  // --- 第一遍扫描：分配标签并记录邻域冲突 ---
  for (uint8 col = 0; col < MT9V03X_H; col++) {
    for (uint8 row = 0; row < MT9V03X_W; row++) {
      if (img[col][row] == 255) { // 仅处理白色像素
        uint8 up = (col > 0) ? copy_label[col - 1][row] : 0;
        uint8 left = (row > 0) ? copy_label[col][row - 1] : 0;
        uint8 final_label;
        
        if (up == 0 && left == 0) {
          // 情况1：孤立点，分配新标签
          if (num_flag < 250) num_flag++;
          final_label = num_flag;
        } else if (up != 0 && left == 0) {
          // 情况2：仅上方有标签，继承
          final_label = up;
        } else if (up == 0 && left != 0) {
          // 情况3：仅左方有标签，继承
          final_label = left;
        } else {
          // 情况4：上方左方均有标签，继承上方并记录冲突合并
          final_label = up;
          if (up != left) UF_Union(up, left);
        }
        copy_label[col][row] = final_label;
        
        // 累加统计数据：面积、坐标和（用于算质心）
        stats[final_label].area++;
        stats[final_label].sum_x += row;
        stats[final_label].sum_y += col;
      }
    }
  }
  
  // --- 第二遍扫描：根据并查集结果合并标签数据 ---
  for (int i = num_flag; i >= 1; i--) {
    uint8 root = UF_Find(i);
    if (root != i) {
      stats[root].area += stats[i].area;
      stats[root].sum_x += stats[i].sum_x;
      stats[root].sum_y += stats[i].sum_y;
      stats[i].area = 0; // 清空被合并的标签面积
    }
  }
  return num_flag;
}

/**
* @brief 质心计算
* @details 将累加的坐标和除以面积，得到每个连通域的几何中心
* @param num_flag 标签总数
* @param out_list 输出的目标信息结构体数组
* @return uint8 过滤噪点后的有效目标数量
*/
uint8 Calculate_Centroids(uint8 num_flag, Target_t* out_list) {
  uint8 count = 0;
  for (int i = 1; i <= num_flag; i++) {
    // 面积阈值过滤（过滤掉小于5像素的噪点）
      out_list[count].x =stats[i].sum_x / stats[i].area;
      out_list[count].y =stats[i].sum_y / stats[i].area;
      out_list[count].area = stats[i].area;
      count++;
  }
  return count;
}

/**
* @brief 目标排序
* @details 使用选择排序算法，按连通域面积从大到小对目标进行重排
* @param list 目标结构体数组
* @param count 目标数量
*/
void Sort_Targets_By_Area(Target_t* list, uint8 count) {
  for (int i = 0; i < (int)count - 1; i++) {
    int max_idx = i;
    for (int j = i + 1; j < (int)count; j++) {
      if (list[j].area > list[max_idx].area) max_idx = j;
    }
    if (max_idx != i) {
      Target_t temp = list[i];
      list[i] = list[max_idx];
      list[max_idx] = temp;
    }
  }
}

/**
* @brief 径向畸变矫正
* @details 基于 Brown-Conrady 模型（K1, K2 参数）矫正镜头引起的桶形或枕形畸变
* @param list 目标列表（函数会直接修改其内部的 x, y 坐标）
* @param count 目标数量
*/
void Apply_Distortion_Correction(Target_t* list, uint8 count,float height_mm) {
  for (int i = 0; i < count; i++) 
  {
    // 1. 坐标归一化（将坐标映射到以图像中心为原点的相对范围）
    float nx = (list[i].x - CX) / FX;
    float ny = (list[i].y - CY) / FY;
    
    // 2. 计算点到中心的平方距离 r^2
    float r2 = nx * nx + ny * ny;
    
    // 3. 计算畸变因子 f = 1 + k1*r^2 + k2*r^4
    float f = 1.0f + K1 * r2 + K2 * r2 * r2;
    
    // 得到矫正后的理想归一化坐标
    list[i].nx_corr = nx * f; 
    list[i].ny_corr = ny * f;
    
    list[i].world_x = list[i].nx_corr * height_mm;
    list[i].world_y = list[i].ny_corr * height_mm;
    
  }
}

/**
* @brief 逆透视处理封装
* @details 遍历目标列表，将每个目标的像素坐标转换为世界物理坐标
* @param list 目标列表
* @param count 目标数量
* @param xb 输出：转换后最后一个目标的物理坐标X
* @param yb 输出：转换后最后一个目标的物理坐标Y
*/
void Apply_Inverse_Perspective(Target_t* list, uint8 count, float* xb, float* yb) {
  for (int i = 0; i < count; i++) {
    homography_transform(list[i].x, list[i].y, xb, yb);
  }
}

#define CAR_SIDE_DIST      280.0f  // 腰长：车头到车尾灯的距离 (mm)
#define CAR_BASE_DIST      140.0f   // 底边：车尾两个灯之间的距离 (mm)
#define CAR_GEOM_ERR       50.0f   // 物理尺寸允许误差 (mm)

/**
* @brief 等腰三角形锁车并识别多信标
* @details 寻找符合物理尺寸的等腰三角形，Head放在list末尾，两个Tail放在倒数2,3位
* @param list 目标列表 (需已完成畸变矫正和world坐标换算)
* @param count 目标数量指针
* @return uint8 1: 锁车成功, 0: 失败
*/
uint8 Identify_TriangleCar_And_Beacons(Target_t* list, uint8* count) {
  if (*count < 4) return 0; // 至少需要 1个信标 + 3个车灯
  
  uint8 found_car = 0;
  uint8 b_h = 0, b_t1 = 0, b_t2 = 0; // 记录最优三角形的索引
  float min_error = 1000.0f;
  
  // --- 步骤 1: 全局三回路搜索最优等腰三角形 ---
  // 跳过可能是信标的大点（假设信标排在前面，从索引0开始找也可以，逻辑会自动排除）
  for (uint8 i = 0; i < *count; i++) {
    for (uint8 j = i + 1; j < *count; j++) {
      for (uint8 k = j + 1; k < *count; k++) {
        
        if(list[i].area<10)
        {
          // 计算三边长 (物理距离 mm)
          float d_ij = sqrtf(powf(list[i].world_x - list[j].world_x, 2) + powf(list[i].world_y - list[j].world_y, 2));
          float d_ik = sqrtf(powf(list[i].world_x - list[k].world_x, 2) + powf(list[i].world_y - list[k].world_y, 2));
          float d_jk = sqrtf(powf(list[j].world_x - list[k].world_x, 2) + powf(list[j].world_y - list[k].world_y, 2));
          
          // 寻找等腰对：比较三对边的差值
          float diff_i = fabsf(d_ij - d_ik); // i是顶点的误差
          float diff_j = fabsf(d_ij - d_jk); // j是顶点的误差
          float diff_k = fabsf(d_ik - d_jk); // k是顶点的误差
          
          uint8 curr_h, curr_t1, curr_t2;
          float side_err, base_dist;
          
          // 逻辑判定：哪一个点作为顶点时，两条腰最接近相等
          if (diff_i < diff_j && diff_i < diff_k) {
            curr_h = i; curr_t1 = j; curr_t2 = k;
            side_err = diff_i; base_dist = d_jk;
          } else if (diff_j < diff_i && diff_j < diff_k) {
            curr_h = j; curr_t1 = i; curr_t2 = k;
            side_err = diff_j; base_dist = d_ik;
          } else {
            curr_h = k; curr_t1 = i; curr_t2 = j;
            side_err = diff_k; base_dist = d_ij;
          }
          
          //                // 校验：1. 顶点必须在中心安全区 2. 物理尺寸符合预设
          //                float r_h = sqrtf(powf(list[curr_h].x - CX, 2) + powf(list[curr_h].y - CY, 2));
          //                if (r_h < CENTER_SAFE_RADIUS && side_err < CAR_GEOM_ERR) {
          //                    // 进一步校验底边长度是否符合
          if (fabsf(base_dist - CAR_BASE_DIST) < CAR_GEOM_ERR) 
          {
            if (side_err < min_error) {
              min_error = side_err;
              b_h = curr_h; b_t1 = curr_t1; b_t2 = curr_t2;
              found_car = 1;
            }
            // }
          }
        }
      }
    }
  }
  
  if (!found_car) return 0;
  
  // --- 步骤 2: 提取车灯，重排列表 ---
  Target_t head_node = list[b_h];
  Target_t tail1_node = list[b_t1];
  Target_t tail2_node = list[b_t2];
  
  Target_t valid_beacons[MAX_TARGETS];
  uint8 beacon_num = 0;
  
  for (uint8 m = 0; m < *count; m++) {
    if (m == b_h || m == b_t1 || m == b_t2) continue;
    if (list[m].area >= BEACON_MIN_AREA) {
      valid_beacons[beacon_num++] = list[m];
    }
  }
  
  // 重构 list: [Beacon0, Beacon1... Tail1, Tail2, Head]
  for (uint8 n = 0; n < beacon_num; n++) list[n] = valid_beacons[n];
  list[beacon_num]     = tail1_node; // count - 3
  list[beacon_num + 1] = tail2_node; // count - 2
  list[beacon_num + 2] = head_node;  // count - 1
  
  *count = beacon_num + 3;
  return 1;
}