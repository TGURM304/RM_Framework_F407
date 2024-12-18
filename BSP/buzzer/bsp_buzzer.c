//
// Created by fish on 2024/12/13.
//

#include "bsp_buzzer.h"

#include "bsp_tim.h"
#include "cmsis_os2.h"

void bsp_buzzer_init() {
    HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}

void bsp_buzzer_alarm(double freq, double blank) {
    bsp_tim_config(&htim4, freq);
    bsp_tim_set_blank(&htim4, TIM_CHANNEL_3, blank);
}

void bsp_buzzer_quiet() {
    bsp_tim_set_blank(&htim4, TIM_CHANNEL_3, 0);
}

void bsp_buzzer_flash(double freq, double blank, uint32_t duration) {
    bsp_buzzer_alarm(freq, blank);
    osDelay(duration);
    bsp_buzzer_quiet();
}