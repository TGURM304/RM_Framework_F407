//
// Created by fish on 2024/11/16.
//

#pragma once

#ifdef __cplusplus

void Damiao_test();
void Damiao_enable();
extern "C" {
#endif

void app_chassis_init();
void app_chassis_task(void *argument);

#ifdef __cplusplus
}
#endif