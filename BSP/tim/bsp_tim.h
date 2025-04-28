//
// Created by fish on 2024/12/13.
//

#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "tim.h"

#define BSP_TIM_LIMIT 25

/*!
 * 设置时钟设备原始参数 (意义同 CubeMX 中相关参数)
 * @param h 时钟设备
 * @param period 周期
 * @param prescaler 预分频系数
 */
void bsp_tim_set(TIM_HandleTypeDef *h, uint16_t period, uint16_t prescaler);

/*!
 * 配置时钟设备到给定频率
 * @param h 时钟设备
 * @param p 频率 (Hz)
 */
void bsp_tim_config(TIM_HandleTypeDef *h, double p);

/*!
 * 设置 PWM 输出占空比
 * @param h 时钟设备
 * @param channel PWM 通道
 * @param blank 占空比
 */
void bsp_tim_set_blank(TIM_HandleTypeDef *h, uint32_t channel, double blank);

/*!
 * 使能 PWM 输出
 * @param h 时钟设备
 * @param channel PWM 通道
 */
void bsp_tim_pwm_enable(TIM_HandleTypeDef *h, uint32_t channel);

/*!
 * 关闭 PWM 输出
 * @param h 时钟设备
 * @param channel PWM 通道
 */
void bsp_tim_pwm_disable(TIM_HandleTypeDef *h, uint32_t channel);

/*!
 * 用于在 main.c 中调用的 callback，禁止用户调用
 * @param h 时钟设备
 */
void bsp_tim_callback(TIM_HandleTypeDef *h);

/*!
 * 用于使能定时中断
 * @param h 时钟设备
 * @param f 中断回调函数
 */
void bsp_tim_it_enable(TIM_HandleTypeDef *h, void (*f)());

#endif //BSP_TIM_H
