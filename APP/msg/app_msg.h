//
// Created by fish on 2024/11/17.
//

#ifndef APP_MSG_H
#define APP_MSG_H

#include "bsp_uart.h"
#include <cstdint>
#include <initializer_list>

#define APP_MSG_VOFA_CHANNEL_LIMIT 10

void app_msg_vofa_send(bsp_uart_e e, std::initializer_list <double> f);

//放置双板通信所必需的结构体及其成员
struct __attribute__((packed)) app_msg_mcu_t {
	int16_t tran_msg[4];

};

#endif //APP_MSG_H
