//
// Created by fish on 2024/11/17.
//

#ifndef APP_MSG_H
#define APP_MSG_H

#include "bsp_uart.h"
#include <cstdint>
#include <initializer_list>

#define APP_MSG_VOFA_CHANNEL_LIMIT 10

/*!
 * 通过 Vofa+ 的 Justfloat 协议发送调试数据
 * @param e 串口枚举类
 * @param f 待发送数据列表 (数据需为 float 或 double)
 * @note app_msg_vofa_send(E_UART_DEBUG, { data1, data2, data3 });
 */
void app_msg_vofa_send(bsp_uart_e e, std::initializer_list <double> f);

#endif //APP_MSG_H
