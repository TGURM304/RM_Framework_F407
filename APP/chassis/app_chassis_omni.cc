//
// Created by fish on 2024/11/16.
//

#include "app_chassis.h"

#include "app_motor.h"
#include "bsp_uart.h"
#include "sys_task.h"
#include "bsp_rc.h"
#include "app_conf.h"
#include "app_ins.h"
#include "app_msg.h"
#include "app_sys.h"
#include "bsp_can.h"

#include <cmath>
#include <cstring>
#include "bsp_def.h"
/*
 *  适用于全向轮（全向轮步兵、哨兵）
 *  实现了基础的旋转、平移
 */

/*
 *  全向轮
 *  ^ vy
 *  |       LU              RU
 *  |           / ------ \
 *  |       C   |        |
 *  |           |        |
 *  |           \ ------ /
 *  |       LD              RD
 *  O------------------------------> vx
 *
 *  不改变轮子正方向（默认为顺时针方向）
 *
 *  v_LU = rotate + vy * sqrt(2) + vx * sqrt(2)
 *  v_RU = rotate - vy * sqrt(2) + vx * sqrt(2)
 *  v_RD = rotate - vy * sqrt(2) - vx * sqrt(2)
 *  v_LD = rotate + vy * sqrt(2) - vx * sqrt(2)
 *
 *  对于全向轮步兵来说，该视图下 C板 在 LU 和 LD 之间
 */

#ifdef COMPILE_CHASSIS_OMNI

OS::Task chassis_task;

MotorController <DJIMotor> LU("chassis_left_up", DJIMotor::M3508, { .id = 0x02, .port = E_CAN1, .mode = DJIMotor::CURRENT },
                             PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> RU("chassis_right_up", DJIMotor::M3508, { .id = 0x01, .port = E_CAN1, .mode = DJIMotor::CURRENT },
                             PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> RD("chassis_left_down", DJIMotor::M3508, { .id = 0x04, .port = E_CAN1, .mode = DJIMotor::CURRENT },
                             PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});
MotorController <DJIMotor> LD("chassis_right_down", DJIMotor::M3508, { .id = 0x03, .port = E_CAN1, .mode = DJIMotor::CURRENT },
                             PID_SPEED, { .Kp = 8.5, .Ki = 0.01, .Kd = 0.01, .out_limit = 16384, .iout_limit = 1000 }, {});


// 直角坐标系下的底盘速度，符合人类直觉，y 轴正方向为机体前进方向。
double vx = 0, vy = 0;
// 旋转速度
double rotate;
//存储6020角度值
int16_t rx_6020_msg[4] = {0};





// 双板通信（发送）
void msg_mcu_send_c () {
	app_msg_mcu_t data_mcu_chassis_send = {0 };
	BSP_ASSERT(sizeof(app_msg_mcu_t) == 8);
	data_mcu_chassis_send.tran_msg[0] = bsp_rc_data()->rc_r[0];
	data_mcu_chassis_send.tran_msg[1] = bsp_rc_data()->rc_r[1];
	data_mcu_chassis_send.tran_msg[2] = bsp_rc_data()->s_l;
	data_mcu_chassis_send.tran_msg[3] = bsp_rc_data()->s_r;
	bsp_can_send(E_CAN2, 0x123, reinterpret_cast <uint8_t *> (&data_mcu_chassis_send));
}

//双板通信回调
app_msg_mcu_t data_rx_gimbal;
void gimbal_callback(bsp_can_msg_t *msg) {
	if (msg->header.StdId != 0x456) return;
	memcpy(&data_rx_gimbal, msg->data, sizeof data_rx_gimbal);
}




//获取6020角度的回调函数
void can_get_yaw_moto_angle(bsp_can_msg_t * message)
{
	rx_6020_msg[0]=(int16_t)(message->data[0] << 8 | message->data[1]);

}


// 静态任务，在 CubeMX 中配置
void app_chassis_task(void *argument) {
	// Wait for system init.
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);

	//设置接受6020对应的can的回调，用于获取6020真实角度值
	bsp_can_set_callback(E_CAN2, 0x205, can_get_yaw_moto_angle);

	//    //设置接收云台传回来的can的回调
	bsp_can_set_callback(E_CAN2, 0x456, gimbal_callback);

	while(true) {

		//给云台发信息
		msg_mcu_send_c();

		vx = 1.0 * bsp_rc_data()->rc_l[0] * 2, vy = 1.0 * bsp_rc_data()->rc_l[1] * 2;
		//        if(bsp_rc_data()->s_l == 1) rotate = 2000;
		//        else if(bsp_rc_data()->s_l == -1) rotate = -2000;
		//        else rotate = 5.0 * bsp_rc_data()->rc_r[0];
		rotate = 2.5 *bsp_rc_data()->reserved;


		double r = std::sqrt(vx * vx + vy * vy), theta = std::atan2(vy, vx);
		theta += rx_6020_msg[0] / 22.7f / 180 * M_PI;  //折算6020位置编码器值作为角度值解算小陀螺
		vx = r * std::cos(theta), vy = r * std::sin(theta);


		LU.target = rotate + vy * M_SQRT2 + vx * M_SQRT2;
		RU.target = rotate - vy * M_SQRT2 + vx * M_SQRT2;
		RD.target = rotate - vy * M_SQRT2 - vx * M_SQRT2;
		LD.target = rotate + vy * M_SQRT2 - vx * M_SQRT2;





		app_msg_vofa_send(E_UART_DEBUG,{
		                                    LU.device()->status.speed,
		                                    LD.device()->status.speed,
		                                    RD.device()->status.speed,
		                                    RU.device()->status.speed,
		                                    (float)rx_6020_msg[0] / 22.7f,
		                                    INS::data()->yaw,
		                                    (float)data_rx_gimbal.tran_msg[0], //接收到的陀螺仪数据
		                                    (float)data_rx_gimbal.tran_msg[1],
		                                    (float)data_rx_gimbal.tran_msg[2],
		                                    (float)data_rx_gimbal.tran_msg[3]
		                                });

		OS::Task::SleepMilliseconds(1);
	}
}

void app_chassis_init() {
	LU.init(); LD.init(); RU.init(); RD.init();
	// LU.relax(); LD.relax(); RU.relax(); RD.relax();
}

#endif
