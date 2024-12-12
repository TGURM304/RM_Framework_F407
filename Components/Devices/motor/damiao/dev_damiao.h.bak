//
// Created by 15082 on 2024/12/7.
//

#ifndef DEV_DAMIAO_H
#define DEV_DAMIAO_H

#include "bsp_can.h"

#define DAMIAO_MOTOR_LIMIT 4

class DamiaoMotor {
public:
	DamiaoMotor() = default;
	enum control_mode_e {
		E_MIT = 0,
		E_Position = 1,
		E_Speed = 2,
		E_MITE = 3
	};
	enum motor_type_e {
		E_4310,
		E_4340
	};
	struct Param {
		uint16_t slave_id;
		uint16_t master_id;
		bsp_can_e port;
		control_mode_e mode;
		float max_speed, max_position;
	};
	DamiaoMotor(const Param& param);
	void MIT_damiao_update(float position, float speed, float position_p, float position_d, float T);
	void can_send();
	void MIT_enable();
	void Speed_update(float speed);
private:
	bsp_can_e port;
	control_mode_e mode;
	float max_position, max_speed;
	uint16_t receive_id,send_id;

	uint16_t P_des, V_des, Kp, Kd, T_ff;
	uint8_t can_ary[8];

};

#endif //DEV_DAMIAO_H
