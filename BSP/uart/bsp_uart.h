//
// Created by fish on 2024/9/2.
//

#ifndef BSP_UART_H
#define BSP_UART_H

#include "usart.h"
#include "stdarg.h"

#define UART_BUFFER_SIZE 1024

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * 串口设备枚举类
 */
typedef enum {
    E_UART_DEBUG, // E_UART1
    E_UART_RC,
    E_UART_VISION,
    E_UART_END
} bsp_uart_e;

#define UART_ENUM_SIZE (E_UART_END + 1)

/*!
 * 初始化串口设备
 * @param e 串口设备枚举类
 * @param h 对应 HAL 库设备的指针
 */
void bsp_uart_init(bsp_uart_e e, UART_HandleTypeDef *h);

/*!
 * 串口发送
 * @param e 串口设备枚举类
 * @param s 待发送数据起始指针
 * @param l 待发送数据长度
 */
void bsp_uart_send(bsp_uart_e e, uint8_t *s, uint16_t l);

/*!
 * 串口打印
 * @param e 串口设备枚举类
 * @param fmt 同 printf
 * @param ... 同 printf
 * @note bsp_uart_printf(E_UART_DEBUG, "%d", num);
 */
void bsp_uart_printf(bsp_uart_e e, const char *fmt, ...);

/*!
 * 设置串口回调函数，当对应串口收到消息时回调函数会被调用
 * @param e 串口设备枚举类
 * @param f 串口回调函数
 */
void bsp_uart_set_callback(bsp_uart_e e, void (*f)(bsp_uart_e e, uint8_t *s, uint16_t l));

#ifdef __cplusplus
}
#endif

#endif //BSP_UART_H
