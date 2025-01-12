//
// Created by fish on 2024/12/13.
//

#ifndef BSP_TIM_H
#define BSP_TIM_H

#include "tim.h"

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

#endif //BSP_TIM_H
