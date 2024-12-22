//
// Created by 15082 on 2024/12/22.
//

#ifndef ALG_SMART_PID_DEV_H
#define ALG_SMART_PID_DEV_H

#include "main.h"

namespace Smart_PID{
typedef  enum {
	Derivative_On_Measurement = 0x01,   //微分先行
	Trapezoid_Integral = 0x02,			//梯形积分
	OutputFilter = 0x04,                //输出滤波
	ChangingIntegralRate = 0x08,        //变积分，变积分函数输出修正系数0~1，实现积分的平缓化
	IntegralSaturate = 0x10				//积分抗饱和
}control_mode_e;
enum {
	Disable,
	Enable
};
class PID {
	public:
	PID();
	PID(float p, float i, float d);
	PID(float p, float i, float d, float max_sum, float max_i,
		uint16_t ctr,
		float (*f)(float err), float (*flt)(float output));

	float PID_calculate(float current, float target);
	float Smart_PID_calculate(float current, float target);

	private:
	float Kp,Ki,Kd;
	float P_out,I_out,D_out,Last_I_out;
	float I_max,Sum_max;
	float input, last_input;
	float err, last_err;
	float sum;
	float out_put,active_flag;
	float (*changing_integral)(float err);
	float (*filter)(float output);
	uint16_t ctr_code;
};
}

#endif //ALG_SMART_PID_DEV_H
