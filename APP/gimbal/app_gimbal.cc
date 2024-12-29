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
        OS::Task::SleepMilliseconds(2);
    }
}

void app_gimbal_init() {

}

#endif