//
// Created by 15082 on 2024/12/10.
//

#include "app_free_motor.h"
#include "cmath"

Algorithm::PID yaw_speed(Algorithm::PID::POSITION,80,0.0,10,18000,2000);
Algorithm::PID yaw_degrees(Algorithm::PID::POSITION,5,0,2,720,30);

static OS::Task FreeMotorTask;

double set_speed = 0,set_voltage = 0;
static auto ins = INS::data();
double last_speed = 0;
double last_degrees = 0;
int count = 0;

/*
 * 在PID基础上添加了前馈的代码，实现了更快的响应
 */
double PID_forward_feed(double current_degree, double set_degree) {
	set_speed = yaw_degrees.update(current_degree,set_degree);
	set_speed += (set_degree - last_degrees)*10;//角度输出速度的一阶前馈
	last_degrees = current_degree;
	/*set_speed = sin((count)*3.14/1000)*200;
	count++;*///正弦转测试代码
	set_voltage = yaw_speed.update(ins->dt_yaw*1000,set_speed);
	set_voltage += (set_speed - last_speed)*50;//速度输出电压的一阶前馈
	last_speed = set_speed;
	return set_voltage;
}
