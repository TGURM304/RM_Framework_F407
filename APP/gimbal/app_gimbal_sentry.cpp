//
// Created by 15082 on 2024/12/10.
//

#include <cstdio>
#include <cstring>

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
#include "bsp_def.h"

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
}
/*
 * 调试用接口，使用串口修改速度环PID参数
 */
void set_target(bsp_uart_e e, uint8_t *s, uint16_t l) {
	float P,I,D;
	sscanf((char *) s, "%f,%f,%f", &P, &I, &D);
	bsp_uart_printf(E_UART_DEBUG,"P:%f,I:%f,D:%f",P,I,D);
	yaw_speed.clear();
	yaw_speed.set_para(P, I, D);
}

double filter[10], filter_sum; uint8_t ptr = 0;

static const auto ins = INS::data();
static const auto rc = bsp_rc_data();



//双板通信发送
void msg_mcu_send_g() {
	BSP_ASSERT(sizeof(app_msg_mcu_t) == 8);
	app_msg_mcu_t data_mcu_gimbal_send = {0};
	data_mcu_gimbal_send.tran_msg[0] = INS::data()->yaw;//需注意yaw的角度是否能存在uint16_t里，会不会溢出
	data_mcu_gimbal_send.tran_msg[1] = 0;
	data_mcu_gimbal_send.tran_msg[2] = 0;
	data_mcu_gimbal_send.tran_msg[3] = 0;
	bsp_can_send(E_CAN2, 0x456, reinterpret_cast <uint8_t *> (&data_mcu_gimbal_send));
}

// 双板通信回调
app_msg_mcu_t data_rx_chassis;
void chassis_callback(bsp_can_msg_t *msg) {
	BSP_ASSERT(msg->header.StdId == 0x123);
	memcpy(&data_rx_chassis, msg->data, sizeof data_rx_chassis);
}



void app_gimbal_task(void *argument) {
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

	//调试接口，现已注释
	/*bsp_uart_set_callback(E_UART_DEBUG, set_target);*/

	motor.reset();
	OS::Task::SleepMilliseconds(50);
	motor.enable();

	//设置双板通信回调
	bsp_can_set_callback(E_CAN2,0X123,chassis_callback);

	while(true) {

		//发送双板通信数据
		msg_mcu_send_g();

		ptr = (ptr + 1) % 10;

		filter_sum -= filter[ptr];
		filter_sum += (filter[ptr] = PID_forward_feed(ins->yaw, data_rx_chassis.tran_msg[0]/5.0));

		m_yaw.update(static_cast <float> (filter_sum / 10));

		app_msg_vofa_send(E_UART_DEBUG, {
			rc->rc_r[0]/5.0,
			ins->yaw,
			ins->dt_yaw*1000,
			set_speed
		});

		motor.control(0, 1);
		OS::Task::SleepMilliseconds(1);
	}
}

#endif