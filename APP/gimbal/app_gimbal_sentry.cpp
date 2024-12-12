//
// Created by 15082 on 2024/12/10.
//

#include <cstdio>

#include "app_conf.h"
#include "app_sys.h"
#include "app_conf.h"
#include "app_ins.h"
#include "app_motor.h"
#include "app_msg.h"
#include "bsp_rc.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "dev_motor_dm.h"
#include "app_gimbal.h"
#include "app_free_motor.h"

#ifdef SENTRY_GIMBAL_DEV

DMMotor motor("dm-j4310", DMMotor::J4310, {
	.slave_id = 0x02, .master_id = 0x01, .port = E_CAN1, .mode = DMMotor::POSITION_SPEED,
	.p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
DJIMotor m_yaw(
	"gimbal_yaw",
	DJIMotor::GM6020,
	{ .id = 0x01, .port = E_CAN2, .mode = DJIMotor::VOLTAGE }
);

void app_gimbal_init() {
	m_yaw.init();
	motor.reset();
	OS::Task::SleepMilliseconds(100);
	motor.enable();
}

static auto ins = INS::data();

void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
	float P,I,D;
	sscanf((char *) s, "%f,%f,%f", &P, &I, &D);
	bsp_uart_printf(E_UART_DEBUG,"P:%f,I:%f,D:%f",P,I,D);
	yaw_speed.clear();
	yaw_speed.set_para(P, I, D);
}

float ary_sum(float const *ary, int len) {
	float sum = 0;
	for (int i = 0; i < len; i++) {
		sum += ary[i];
	}
	return sum/len;
}

void app_gimbal_task(void *argument) {
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
	float filter[10];
	int ptr = 0;
	OS::Task::SleepSeconds(2);
	static int count = 0;
	bsp_uart_set_callback(E_UART_DEBUG, set_target);
	auto rc = bsp_rc_data();
	while(true) {
		ptr++;
		filter[ptr] = (float)PID_forward_feed(ins->yaw,rc->rc_r[0]/5.0);
		ptr %= 9;
		m_yaw.update(ary_sum(filter, 10));
		count %=5;
		count++;
		if(count == 5)
		app_msg_vofa_send(E_UART_DEBUG,{
			(float)(rc->rc_r[0]/5.0),
				(float)ins->yaw,
				ins->dt_yaw*1000,
				(float)set_speed
		});
		OS::Task::SleepMilliseconds(1);
	}
}

#endif