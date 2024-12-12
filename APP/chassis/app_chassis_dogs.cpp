//
// Created by 15082 on 2024/12/8.
//

#include "dev_damiao.h.bak"
#include "cmsis_os2.h"
#include "sys_task.h"
#include "app_chassis.h"
#include "app_sys.h"
#include "bsp_uart.h"
#include "app_conf.h"

#ifdef COMPILE_CHASSIS_DOGS

DamiaoMotor LF({
	.slave_id = 8,
	.master_id = 4,
	.port = E_CAN1,
	.mode = DamiaoMotor::E_MIT,
	.max_speed = 45,
	.max_position = 12.5,
})  ;


void Damiao_enable() {
	LF.MIT_enable();
}

void Damiao_test() {
	LF.MIT_damiao_update(12.5,0,15,0.3,0);
	LF.can_send();
	osDelay(2000);
	LF.MIT_damiao_update(-12.5,0,15,0.3,0);
	LF.can_send(); 
	osDelay(2000);
}

OS::Task chassis_task;

void app_chassis_init() {
	Damiao_enable();
}

void app_chassis_task(void *argument) {
	while(!app_sys_ready()) OS::Task::SleepMilliseconds(10);
	for(;;) {
		bsp_uart_printf(E_UART_DEBUG,"Test\r\n");
		Damiao_test();
		osDelay(1);
	}
}
#endif

