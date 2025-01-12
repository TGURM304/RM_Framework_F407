//
// Created by fish on 2024/12/13.
//

#ifndef BSP_BUZZER_H
#define BSP_BUZZER_H

#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/*!
 * 蜂鸣器初始化
 */
void bsp_buzzer_init();

/*!
 * 开启蜂鸣器
 * @param freq 频率
 * @param blank 占空比
 */
void bsp_buzzer_alarm(double freq, double blank);

/*!
 * 关闭蜂鸣器
 */
void bsp_buzzer_quiet();

/*!
 * 开启蜂鸣器一段时间
 * @param freq 频率
 * @param blank 占空比
 * @param duration 持续时间 (ms)
 */
void bsp_buzzer_flash(double freq, double blank, uint32_t duration);

#ifdef __cplusplus
}
#endif

#endif //BSP_BUZZER_H
