//
// Created by fish on 2024/11/3.
//

#pragma once
#include "main.h"

bool app_sys_ready();

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * 系统任务
 */
void app_sys_task();
__weak void app_chassis_task(void *args);
__weak void app_gimbal_task(void *args);
__weak void dev_dji_motor_task(void *args);
__weak void app_ins_task(void *args);

#ifdef __cplusplus
}
#endif