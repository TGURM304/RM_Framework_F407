//
// Created by 15082 on 2024/12/10.
//

#ifndef APP_FREE_MOTOR_H
#define APP_FREE_MOTOR_H

#include "sys_task.h"
#include "alg_pid.h"
#include "app_ins.h"

double PID_forward_feed(double current_degree, double set_degree);

extern Algorithm::PID yaw_speed;
extern Algorithm::PID yaw_degrees;

extern double set_speed ,set_voltage;

#endif //APP_FREE_MOTOR_H
