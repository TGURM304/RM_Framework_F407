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

#define PITCH_MAX 40.00
#define PITCH_MIN (-30.00)
#define PITCH_SPEED 1
DMMotor m_pitch("dm-j4310", DMMotor::J4310, {
	.slave_id = 0x02, .master_id = 0x01, .port = E_CAN1, .mode = DMMotor::POSITION_SPEED,
	.p_max = 12.5, .v_max = 30, .t_max = 10, .kp_max = 500, .kd_max = 5
});
DJIMotor m_yaw(
	"gimbal_yaw",
	DJIMotor::GM6020,
	{ .id = 0x01, .port = E_CAN2, .mode = DJIMotor::VOLTAGE }
);

MotorController<DJIMotor> left_shoot_trigger(
	"left_trigger",
	DJIMotor::M2006,
	{.id = 0x07,.port = E_CAN1,.mode = DJIMotor::CURRENT
	},
	PID_SPEED,
{ .Kp = 10.0, .Ki = 1.0, .Kd = 0.0, .out_limit = 25000, .iout_limit = 20000 },
{ .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 15, .iout_limit = 1 }
);

MotorController<DJIMotor> left_shoot_boost_up(
	"left_shoot_boost_up",
	DJIMotor::M3508,
	{.id = 0x04,.port = E_CAN1,.mode = DJIMotor::CURRENT
	},
	PID_SPEED,
{ .Kp = 20.0, .Ki = 0.5, .Kd = 0.0, .out_limit = 25000, .iout_limit = 20000 },
{ .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 15, .iout_limit = 1 }
);
MotorController<DJIMotor> left_shoot_boost_down(
	"left_shoot_boost_down",
	DJIMotor::M3508,
	{.id = 0x01,.port = E_CAN1,.mode = DJIMotor::CURRENT
	},
	PID_SPEED,
{ .Kp = 20.0, .Ki = 0.5, .Kd = 0.0, .out_limit = 25000, .iout_limit = 20000 },
{ .Kp = 0.0, .Ki = 0.0, .Kd = 0.0, .out_limit = 15, .iout_limit = 1 }
);


/*
 * 调试用接口，使用串口修改速度环PID参数
 * 现已注释
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
static float pitch = 0.0f;//pitch轴相对角度，单位rad，水平为零点
static float yaw = 180.0f;

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

/*
 *陀螺仪角度值默认 -180 ~ +180
 * 是的，这是一个足够优秀的函数，你只需要输入0-360度的yaw和限定范围的pitch，你就可以实现控制了（yaw自带优劣弧哦），让我们欢呼吧
 * pitch: 0~360
 * yaw: 0~360
 */
void gimbal_control(float target_pitch, float target_yaw) {
	//测试中出现难以复现的错误，下面的BSP_ASSERT不建议使用
	/*BSP_ASSERT(pitch >= PITCH_MIN && pitch<= PITCH_MAX && yaw >= 0 && yaw <= 360)*/
	//PID err = target - current
	float delta_yaw = target_yaw - ins->yaw +180;//把yaw值转化为0~360度并且求出delta
	//下面两行为求出优劣弧传入pid中控制
	if (delta_yaw > 180) delta_yaw -= 360;
	if (delta_yaw < -180) delta_yaw += 360;
	ptr = (ptr + 1) % 10;
	filter_sum -= filter[ptr];//滤波，防止前馈可能导致的高频震荡
	filter_sum += (filter[ptr] = PID_forward_feed(0, delta_yaw));
	m_pitch.control(target_pitch/360*3.14, PITCH_SPEED);
}
void app_gimbal_init() {
	m_yaw.init();
	left_shoot_trigger.init();
	left_shoot_boost_up.init();
	left_shoot_boost_down.init();
}
void app_gimbal_task(void *argument) {
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

	//yaw轴速度环调试接口，现已注释
	/*bsp_uart_set_callback(E_UART_DEBUG, set_target);*/

	m_pitch.reset();
	OS::Task::SleepMilliseconds(50);
	m_pitch.enable();

	//设置双板通信回调
	bsp_can_set_callback(E_CAN2,0X123,chassis_callback);
	while(true) {

		//发送双板通信数据
		msg_mcu_send_g();
		//防止数据超过设定范围
		yaw -= (float)data_rx_chassis.tran_msg[0]/1400.0f;
		if(yaw > 360) yaw -= 360;
		if(yaw < -360) yaw += 360;
		pitch -= (float)data_rx_chassis.tran_msg[1]/2000.0f;
		if(pitch > PITCH_MAX) pitch = PITCH_MAX;
		if(pitch < PITCH_MIN) pitch = PITCH_MIN;

		/*gimbal_control(pitch, yaw);//云台控制函数

		m_yaw.update(static_cast <float> (filter_sum / 10));*/

		if(data_rx_chassis.tran_msg[2] == 1) {
			//left_shoot_trigger.target = -1500;
            left_shoot_boost_up.target = -8000;
            left_shoot_boost_down.target = 8000;
		}
		else
		{
			left_shoot_trigger.device()->update(0);
			left_shoot_boost_up.target=0;
			left_shoot_boost_down.target=0;
		}
		//调试云台所用调试接口，输出云台角度和速度波形
		app_msg_vofa_send(E_UART_DEBUG, {
			left_shoot_boost_down.speed,
			left_shoot_boost_up.speed,
			8000.0f
		});
		OS::Task::SleepMilliseconds(1);
	}
}

#endif
/*//smart_pid test
#include "alg_smart_pid_dev.h"

DJIMotor test_motor(
	"gimbal_yaw",
	DJIMotor::GM6020,
	{ .id = 0x01, .port = E_CAN1, .mode = DJIMotor::VOLTAGE }
);

float voltage = 0;
void app_gimbal_init() {
	test_motor.init();
}
float out_put = 0;
float test_changing_integral(float err) {
	if(err >= 20)
		return 0;
	return 1;
}
float test_filter_output(float outPut) {
	return 0;
}

Smart_PID::PID pid_test(0,1,0,10000,0,
	Smart_PID::OutputFilter,
	test_changing_integral,test_filter_output);

void app_gimbal_task(void *argument) {
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
	uint8_t temp = 0;
	app_gimbal_init();
	OS::Task::SleepSeconds(1);
	while(true) {
		app_msg_vofa_send(E_UART_DEBUG, {
			(float)test_motor.feedback_.speed,
			50.0f
		});
		test_motor.update(pid_test.Smart_PID_calculate(test_motor.feedback_.speed,100));
		OS::Task::SleepMilliseconds(1);
	}
}*/