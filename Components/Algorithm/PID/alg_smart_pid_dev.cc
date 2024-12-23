//
// Created by 15082 on 2024/12/22.
//

#include "alg_smart_pid_dev.h"

#include <cmath>

#include "bsp_def.h"

using namespace Smart_PID;

PID::PID() {
	Kp = 0;
	Ki = 0;
	Kd = 0;
	active_flag = Enable;
}
PID::PID(float p, float i, float d) {
	Kp = p;
	Ki = i;
	Kd = d;
	active_flag = Enable;
}
PID::PID(float p, float i, float d, float max_sum, float max_i, float delta, uint16_t ctr, float (*f)(float err), float (*flt)(float output)) {
	Kp = p;
	Ki = i;
	Kd = d;
	I_max = max_i;
	Sum_max = max_sum;
	ctr_code = ctr;
	changing_integral = f;
	filter = flt;
	delta_max = delta;
	active_flag = Enable;
}

float PID::PID_calculate(float current, float target) {
	last_err = err;
	err = target - current;

	P_out = err*Kp;

	I_out += err*Ki;

	D_out = (err - last_err)*Kd;

	sum = P_out + I_out + D_out;

	out_put = sum;

	return out_put;
}

float PID::Smart_PID_calculate(float current, float target) {
	last_err = err;
	err = target - current;

	P_out = err*Kp;

	Last_I_out = I_out;
	if(ctr_code & ChangingIntegralRate)//如果使用了变积分函数
	{
		BSP_ASSERT(changing_integral != nullptr)
		if(err * Last_I_out > 0)//判断当前积分是否为积累趋势。
			I_out += err*Ki*changing_integral(err);
	}
	else if(ctr_code & Trapezoid_Integral)//如果使用了梯形积分
		I_out += (last_err + err)*Ki/2;
	else I_out += err*Ki;
	if(I_max != 0) {//如果I_max！=0 视为启用积分限幅
		if(I_out > I_max)
			I_out = I_max;
		else if(I_out < -I_max)
			I_out = -I_max;
	}
	if(ctr_code & IntegralSaturate) {//如果使用积分抗饱和
		if(sum >= 0.9*Sum_max || sum <= -0.9*Sum_max)
			I_out = Last_I_out;
	}

	if(ctr_code & Derivative_On_Measurement) {//如果使用微分先行
		last_input = input;
		input = current;
		D_out = (last_input - input)*Kd;
	}
	else D_out = ( err - last_err)*Kd;

	last_sum = sum;
	sum = P_out + I_out + D_out;
	if(Sum_max != 0) {//如果Sum_max！=0 视为启用输出限幅
		if(sum > Sum_max)
			sum = Sum_max;
		else if(sum < -Sum_max)
			sum = -Sum_max;
	}
	if(ctr_code & OutputFilter) {
		//输出滤波
		BSP_ASSERT(filter != nullptr);
		sum = filter(sum);
	}
	if(ctr_code & DeltaLimit) {//使用改变限幅
		if(last_sum - sum > delta_max) sum = last_sum -delta_max;
		else if(last_sum - sum < -delta_max) sum = last_sum +delta_max;
	}
	out_put = sum;
	return out_put;
}
