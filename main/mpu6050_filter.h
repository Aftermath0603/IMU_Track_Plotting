#ifndef MPU6050_FILTER_H
#define MPU6050_FILTER_H

#ifdef __cplusplus
extern "C" {
#endif

#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

/* Mahony 姿态融合参数 */
#define Kp 2.0f
#define Ki 0.005f

// 初始化滤波算法
void filter_init(void);

// Mahony 更新函数
void mahony_update(float ax, float ay, float az, float gx, float gy, float gz, float dt);

// 获取欧拉角
void get_euler_angles(float *roll, float *pitch, float *yaw);

#ifdef __cplusplus
}
#endif

#endif // MPU6050_FILTER_H
