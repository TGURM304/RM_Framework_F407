//
// Created by fish on 2024/11/16.
//

#pragma once

#include "app_conf.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * 云台初始化 (随系统初始化调用)
 */
void app_gimbal_init();

/*!
 * 云台任务
 * @param args RTOS 任务参数
 */
void app_gimbal_task(void *args);

#ifdef __cplusplus
}
#endif