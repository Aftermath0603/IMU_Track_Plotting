#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "esp_err.h"
#include <math.h>
#include "mpu6050_filter.h"

#define TAG "IMU_TRACK"

/* 硬件配置 - 复用现有引脚 */
#define I2C_MASTER_SDA_IO           GPIO_NUM_7
#define I2C_MASTER_SCL_IO           GPIO_NUM_8
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define I2C_MASTER_TIMEOUT_MS       500

/* MPU6050 寄存器 */
#define MPU6050_ADDRESS             0x68
#define MPU6050_RA_PWR_MGMT_1       0x6B
#define MPU6050_RA_ACCEL_XOUT_H     0x3B
#define MPU6050_RA_GYRO_XOUT_H      0x43
#define MPU6050_RA_TEMP_OUT_H       0x41

/* 工作参数 */
#define SAMPLE_INTERVAL_MS      20      // 50Hz 采样
#define WAKEUP_INTERVAL_MS      2000
#define STABLE_TIME_THRESHOLD_MS 5000
#define GYRO_AUTO_ZERO_COUNT    100

/* 滤波参数 */
#define LPF_ALPHA               0.15f   
#define ANGLE_LPF_ALPHA         0.10f

/* --- 航迹推断新增宏定义 --- */
#define GRAVITY_MS2             9.80665f    // 重力加速度转换
#define ACCEL_DEADZONE          0.02f       // 加速度死区 (g)，过滤静止抖动
#define VELOCITY_DRAG           0.95f       // 速度衰减系数，用于抑制积分漂移
#define STATIONARY_THRESHOLD    0.02f       // 静止检测阈值 (delta_accel)，对应 mpu6050_calib_main.c 中的 0.02f
#define VEL_ZERO_COUNT          25          // 连续 25 次采样（0.5s）静止则速度归零

// 全局偏置与补偿变量
float accel_x_offset = 0, accel_y_offset = 0, accel_z_offset = 0;
float gyro_x_offset  = 0, gyro_y_offset  = 0, gyro_z_offset  = 0;
float temp_ref = 0;
float ax_temp_bias = 0, ay_temp_bias = 0, az_temp_bias = 0;
float gx_temp_bias = 0, gy_temp_bias = 0, gz_temp_bias = 0;

/* --- 航迹推断全局变量 --- */
float world_pos_x = 0.0f;       // 世界坐标 X (单位: m)
float world_pos_y = 0.0f;       // 世界坐标 Y (单位: m)
float world_vel_x = 0.0f;       // 世界速度 VX (单位: m/s)
float world_vel_y = 0.0f;       // 世界速度 VY (单位: m/s)
float total_distance = 0.0f;    // 累计移动距离 (单位: m)
int static_samples = 0;         // 静态采样计数器

// I2C 基础函数 (复用)
static esp_err_t i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
        .clk_flags = 0,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    esp_err_t err = i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C驱动安装失败");
        return err;
    }
    ESP_LOGI(TAG, "I2C 初始化成功");
    return ESP_OK;
}

static esp_err_t mpu_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    return i2c_master_write_to_device(I2C_MASTER_NUM, MPU6050_ADDRESS, buf, 2, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

static esp_err_t mpu_read(uint8_t reg, uint8_t *data, size_t len) {
    return i2c_master_write_read_device(I2C_MASTER_NUM, MPU6050_ADDRESS, &reg, 1, data, len, pdMS_TO_TICKS(I2C_MASTER_TIMEOUT_MS));
}

void mpu6050_wake(void) {
    mpu_write(MPU6050_RA_PWR_MGMT_1, 0x00);
    vTaskDelay(pdMS_TO_TICKS(100));
}

void mpu6050_sleep(void) {
    mpu_write(MPU6050_RA_PWR_MGMT_1, 0x40);
}

float mpu_read_temp(void) {
    uint8_t buf[2];
    mpu_read(MPU6050_RA_TEMP_OUT_H, buf, 2);
    return (((int16_t)buf[0] << 8) | buf[1]) / 340.0f + 36.53f;
}

void mpu6050_calibrate(void) {
    ESP_LOGW(TAG, "=====================================");
    ESP_LOGW(TAG, "  上电校准 → 请保持静止！");
    ESP_LOGW(TAG, "=====================================");
    vTaskDelay(1000);

    int32_t ax_s=0, ay_s=0, az_s=0, gx_s=0, gy_s=0, gz_s=0;
    const int total = 1000;
    for (int i = 0; i < total; i++) {
        uint8_t buf[14];
        mpu_read(MPU6050_RA_ACCEL_XOUT_H, buf, 14);
        ax_s += (int16_t)(buf[0]<<8|buf[1]);
        ay_s += (int16_t)(buf[2]<<8|buf[3]);
        az_s += (int16_t)(buf[4]<<8|buf[5]) - 16384;
        gx_s += (int16_t)(buf[8]<<8|buf[9]);
        gy_s += (int16_t)(buf[10]<<8|buf[11]);
        gz_s += (int16_t)(buf[12]<<8|buf[13]);
        
        if (i % 200 == 0) {
            ESP_LOGI(TAG, "校准进度：%d%%", i * 100 / total);
        }
        vTaskDelay(1);
    }
    accel_x_offset = (float)ax_s / total;
    accel_y_offset = (float)ay_s / total;
    accel_z_offset = (float)az_s / total;
    gyro_x_offset  = (float)gx_s / total;
    gyro_y_offset  = (float)gy_s / total;
    gyro_z_offset  = (float)gz_s / total;
    temp_ref = mpu_read_temp();
    ESP_LOGI(TAG, "=====================================");
    ESP_LOGI(TAG, "  上电校准完成！");
    ESP_LOGI(TAG, "=====================================");
}

void update_temp_comp(void) {
    float dt = mpu_read_temp() - temp_ref;
    ax_temp_bias = dt * 2.0f; ay_temp_bias = dt * 1.2f; az_temp_bias = dt * 0.8f;
    gx_temp_bias = dt * 0.6f; gy_temp_bias = dt * 0.6f; gz_temp_bias = dt * 0.6f;
}

void mpu6050_get_calibrated(float *ax, float *ay, float *az, float *gx, float *gy, float *gz) {
    uint8_t buf[14];
    mpu_read(MPU6050_RA_ACCEL_XOUT_H, buf, 14);
    *ax = ((float)((int16_t)(buf[0]<<8|buf[1])) - accel_x_offset - ax_temp_bias) / 16384.0f;
    *ay = ((float)((int16_t)(buf[2]<<8|buf[3])) - accel_y_offset - ay_temp_bias) / 16384.0f;
    *az = ((float)((int16_t)(buf[4]<<8|buf[5])) - accel_z_offset - az_temp_bias) / 16384.0f;
    *gx = (float)((int16_t)(buf[8]<<8|buf[9])) - gyro_x_offset - gx_temp_bias;
    *gy = (float)((int16_t)(buf[10]<<8|buf[11])) - gyro_y_offset - gy_temp_bias;
    *gz = (float)((int16_t)(buf[12]<<8|buf[13])) - gyro_z_offset - gz_temp_bias;
}

/**
 * @brief 核心算法：二维航迹推断
 * @param body_ax 滤波后的机体X轴加速度 (g)
 * @param body_ay 滤波后的机体Y轴加速度 (g)
 * @param yaw_deg 当前偏航角 (度)
 * @param dt 采样周期 (s)
 * @param is_stationary 是否处于静止状态
 */
void update_dead_reckoning(float body_ax, float body_ay, float yaw_deg, float dt, bool is_stationary) {
    // 1. 静止检测与速度归零策略 (ZUPT)
    if (is_stationary) {
        static_samples++;
        if (static_samples > VEL_ZERO_COUNT) {
            world_vel_x = 0;
            world_vel_y = 0;
        }
        return; // 静止时不进行积分
    }
    static_samples = 0;

    // 2. 机体坐标系 -> 世界坐标系 (2D 旋转)
    // 初始朝向 Y+，Yaw=0 时，机体 Y 对应世界 Y
    float yaw_rad = yaw_deg * M_PI / 180.0f;
    float cos_y = cosf(yaw_rad);
    float sin_y = sinf(yaw_rad);

    // 坐标变换公式：
    // W_ax = B_ax * cos(yaw) - B_ay * sin(yaw)
    // W_ay = B_ax * sin(yaw) + B_ay * cos(yaw)
    float world_ax = body_ax * cos_y - body_ay * sin_y;
    float world_ay = body_ax * sin_y + body_ay * cos_y;

    // 3. 加速度死区处理与单位转换 (g -> m/s^2)
    if (fabsf(world_ax) < ACCEL_DEADZONE) world_ax = 0;
    if (fabsf(world_ay) < ACCEL_DEADZONE) world_ay = 0;
    
    float acc_x_ms2 = world_ax * GRAVITY_MS2;
    float acc_y_ms2 = world_ay * GRAVITY_MS2;

    // 4. 一次积分：获取速度 (V = V0 + a*dt)
    world_vel_x += acc_x_ms2 * dt;
    world_vel_y += acc_y_ms2 * dt;

    // 速度衰减 (抑制积分漂移)
    world_vel_x *= VELOCITY_DRAG;
    world_vel_y *= VELOCITY_DRAG;

    // 5. 二次积分：获取位移 (P = P0 + v*dt)
    float dx = world_vel_x * dt;
    float dy = world_vel_y * dt;
    world_pos_x += dx;
    world_pos_y += dy;

    // 6. 累计路程计算
    total_distance += sqrtf(dx * dx + dy * dy);
}

void app_main(void) {
    ESP_ERROR_CHECK(i2c_master_init());
    mpu6050_wake();
    filter_init();
    mpu6050_calibrate();

    float ax, ay, az, gx, gy, gz;
    float lpf_ax = 0.0f, lpf_ay = 0.0f, lpf_az = 1.0f;
    float lpf_roll = 0.0f, lpf_pitch = 0.0f, lpf_yaw = 0.0f;
    float prev_ax = 0, prev_ay = 0, prev_az = 0;
    float roll, pitch, yaw;
    float dt = SAMPLE_INTERVAL_MS / 1000.0f;
    char vofa_buf[256];

    TickType_t last_motion_tick = xTaskGetTickCount();
    bool is_sleeping = false;
    int stationary_counter = 0;

    ESP_LOGI(TAG, "航迹推断系统已就绪...");

    while (1) {
        if (is_sleeping) {
            vTaskDelay(pdMS_TO_TICKS(WAKEUP_INTERVAL_MS));
            mpu6050_wake();
        }

        mpu6050_get_calibrated(&ax, &ay, &az, &gx, &gy, &gz);

        // 加速度低通滤波
        lpf_ax = LPF_ALPHA * ax + (1.0f - LPF_ALPHA) * lpf_ax;
        lpf_ay = LPF_ALPHA * ay + (1.0f - LPF_ALPHA) * lpf_ay;
        lpf_az = LPF_ALPHA * az + (1.0f - LPF_ALPHA) * lpf_az;

        // 运动强度检测
        float delta_accel = sqrtf(powf(lpf_ax - prev_ax, 2) + powf(lpf_ay - prev_ay, 2) + powf(lpf_az - prev_az, 2));
        float gyro_mag = sqrtf(gx*gx + gy*gy + gz*gz);
        prev_ax = lpf_ax; prev_ay = lpf_ay; prev_az = lpf_az;

        // 动态零偏补偿 (静止时)
        bool is_stationary = (gyro_mag < 50.0f && delta_accel < STATIONARY_THRESHOLD);
        if (is_stationary) {
            stationary_counter++;
            if (stationary_counter > GYRO_AUTO_ZERO_COUNT) {
                gyro_x_offset += gx * 0.05f;
                gyro_y_offset += gy * 0.05f;
                gyro_z_offset += gz * 0.05f;
                stationary_counter = 0;
            }
        } else {
            stationary_counter = 0;
        }

        // 休眠唤醒逻辑
        if (delta_accel > 0.05f || gyro_mag > 300.0f) {
            last_motion_tick = xTaskGetTickCount();
            if (is_sleeping) {
                is_sleeping = false;
                ESP_LOGI(TAG, "唤醒：重置航迹起始点");
                // 唤醒后通常位置不再可靠，可选择重置或续算，此处选择保留位置但速度归零
                world_vel_x = 0; world_vel_y = 0;
            }
        }

        if (!is_sleeping) {
            // 姿态解算
            float gx_rad = (gx / 131.0f) * M_PI / 180.0f;
            float gy_rad = (gy / 131.0f) * M_PI / 180.0f;
            float gz_rad = (gz / 131.0f) * M_PI / 180.0f;
            mahony_update(lpf_ax, lpf_ay, lpf_az, gx_rad, gy_rad, gz_rad, dt);
            get_euler_angles(&roll, &pitch, &yaw);

            // 角度平滑
            lpf_roll  = ANGLE_LPF_ALPHA * roll  + (1.0f - ANGLE_LPF_ALPHA) * lpf_roll;
            lpf_pitch = ANGLE_LPF_ALPHA * pitch + (1.0f - ANGLE_LPF_ALPHA) * lpf_pitch;
            lpf_yaw   = ANGLE_LPF_ALPHA * yaw   + (1.0f - ANGLE_LPF_ALPHA) * lpf_yaw;

            /* --- 执行航迹推断 --- */
            update_dead_reckoning(lpf_ax, lpf_ay, lpf_yaw, dt, is_stationary);

            // 格式化输出 (VOFA+)
            // 字段：LPF_Roll, LPF_Pitch, LPF_Yaw, PosX, PosY, VelX, VelY, Distance
            snprintf(vofa_buf, sizeof(vofa_buf), "%.2f,%.2f,%.2f,%.3f,%.3f,%.3f,%.3f,%.3f\n",
                     lpf_roll, lpf_pitch, lpf_yaw, 
                     world_pos_x, world_pos_y, world_vel_x, world_vel_y, total_distance);
            
            printf("%s", vofa_buf);

            update_temp_comp();

            if (xTaskGetTickCount() - last_motion_tick > pdMS_TO_TICKS(STABLE_TIME_THRESHOLD_MS)) {
                mpu6050_sleep();
                is_sleeping = true;
                ESP_LOGW(TAG, "进入休眠，航迹暂停");
            }
            vTaskDelay(pdMS_TO_TICKS(SAMPLE_INTERVAL_MS));
        }
    }
}
