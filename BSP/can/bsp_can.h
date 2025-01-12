//
// Created by fish on 2024/11/15.
//

#ifndef BSP_CAN_H
#define BSP_CAN_H

#define BSP_CAN_FILTER_LIMIT 25
#define BSP_CAN_MSG_LIMIT 8

#include "stdint.h"
#include "can.h"

#ifdef __cplusplus
extern "C" {
#endif

#define BSP_CAN_ENUM_SIZE E_CAN_END_DONT_REMOVE

/*!
 * CAN 设备枚举类
 */
typedef enum {
    E_CAN1,
    E_CAN2,
    E_CAN_END_DONT_REMOVE
} bsp_can_e;

/*!
 * CAN 消息接收类
 */
typedef struct {
    bsp_can_e port;
    CAN_RxHeaderTypeDef header;
    uint8_t data[BSP_CAN_MSG_LIMIT];
} bsp_can_msg_t;

/*!
 * 初始化 CAN 设备
 * @param e CAN 设备枚举类
 * @param h 对应 HAL 库设备的指针
 */
void bsp_can_init(bsp_can_e e, CAN_HandleTypeDef *h);

/*!
 * 设置 CAN 回调函数，当对应 CAN 设备收到对应 id 消息时会调用回调函数
 * @param e CAN 设备枚举类
 * @param id 消息 id
 * @param f 消息回调函数
 * @return 当前 CAN 设备已注册的回调函数数量
 */
uint8_t bsp_can_set_callback(bsp_can_e e, uint32_t id, void (*f) (bsp_can_msg_t *msg));

/*!
 * 发送一个传统 CAN 包，长度为 8 bytes
 * @param e CAN 设备枚举类
 * @param id 消息 id
 * @param s 数据起始指针
 */
void bsp_can_send(bsp_can_e e, uint32_t id, uint8_t *s);

#ifdef __cplusplus
}
#endif

#endif //BSP_CAN_H
