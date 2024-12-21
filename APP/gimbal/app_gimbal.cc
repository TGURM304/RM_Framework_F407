//
// Created by fish on 2024/11/17.
//

#include "app_gimbal.h"

#include <cstdio>

#include "alg_pid.h"
#include "app_ins.h"
#include "app_msg.h"
#include "sys_task.h"
#include "app_sys.h"
#include "app_vision.h"
#include "dev_motor_dji.h"

#ifdef COMPILE_GIMBAL

// 静态任务，在 CubeMX 中配置
void app_gimbal_task(void *args) {
    // Wait for system init.
    while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

    while(true) {
        vision::send(1, false);
        app_msg_vofa_send(E_UART_DEBUG, {
            vision::recv()->x,
            vision::recv()->y,
            vision::recv()->z,
        });
        OS::Task::SleepMilliseconds(2);
    }
}

void app_gimbal_init() {
    vision::init();
}

#endif