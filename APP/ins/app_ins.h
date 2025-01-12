//
// Created by fish on 2025/1/1.
//

#ifndef APP_INS_H
#define APP_INS_H
#include <cstdint>

#include "bsp_imu.h"

/*!
 * 陀螺仪数据类
 */
struct app_ins_data_t {
	float yaw, pitch, roll;
	bsp_imu_raw_data_t raw;
};

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * 陀螺仪初始化
 */
void app_ins_init();

/*!
 * 陀螺仪任务
 * @param args RTOS 任务参数
 */
void app_ins_task(void *args);

/*!
 * 获取当前陀螺仪状态
 * @return 当前陀螺仪状态
 * @note 0: 等待温控, 1: 等待校准, 2: 正常工作
 */
uint8_t app_ins_status();

/*!
 * 获取陀螺仪数据指针
 * @return 陀螺仪数据指针
 */
const app_ins_data_t *app_ins_data();

#ifdef __cplusplus
}
#endif

#endif //APP_INS_H
