//
// Created by fish on 2024/9/18.
//

#ifndef BSP_IMU_H
#define BSP_IMU_H

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * IMU 原始数据类
 */
typedef struct {
    float gyro[3], accel[3], temp;
} bsp_imu_raw_data_t;

/*!
 * 初始化 IMU
 */
void bsp_imu_init();

/*!
 * 读当前 IMU 数据到给定地址
 * @param data 待写入的 IMU 原始数据类指针
 */
void bsp_imu_read(bsp_imu_raw_data_t *data);

#ifdef __cplusplus
}
#endif

#endif //BSP_IMU_H
