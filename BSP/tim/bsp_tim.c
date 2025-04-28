//
// Created by fish on 2024/12/13.
//

#include "bsp_tim.h"

#include "bsp_def.h"
#include "math.h"

/*
 *  For STM32F407IGH6
 */

#define RCC_APB1_CLOCK_FREQ_MHZ 84
#define RCC_APB2_CLOCK_FREQ_MHZ 168

void bsp_tim_set(TIM_HandleTypeDef *h, uint16_t period, uint16_t prescaler) {
    HAL_TIM_Base_DeInit(h);
    h->Init.Period = period;
    h->Init.Prescaler = prescaler;
    HAL_TIM_Base_Init(h);
}

// usage: bsp_tim_config(&htim1, 1000);
void bsp_tim_config(TIM_HandleTypeDef *h, double p) {
    double t = 1e6 / p, k = 1, period_limit = h->Instance == TIM2 ? (1ll << 32) : (1ll << 16);
    if(h->Instance == TIM1 || h->Instance == TIM8 || h->Instance == TIM9 || h->Instance == TIM10 || h->Instance == TIM11) {
        t *= RCC_APB2_CLOCK_FREQ_MHZ;
    } else {
        t *= RCC_APB1_CLOCK_FREQ_MHZ;
    }
    k = ceil(t / period_limit);
    bsp_tim_set(h, (uint32_t) (t / k) - 1, (uint32_t) k - 1);
}

// 0 <= blank <= 1
// usage: bsp_tim_set_blank(&htim1, TIM_CHANNEL_1, 0.5);
void bsp_tim_set_blank(TIM_HandleTypeDef *h, uint32_t channel, double blank) {
    BSP_ASSERT(0 <= blank && blank <= 1);
    __HAL_TIM_SetCompare(h, channel, (uint32_t) ((h->Init.Period + 1) * blank));
}

void bsp_tim_pwm_enable(TIM_HandleTypeDef *h, uint32_t channel) {
    HAL_TIM_PWM_Start(h, channel);
}

void bsp_tim_pwm_disable(TIM_HandleTypeDef *h, uint32_t channel) {
    HAL_TIM_PWM_Stop(h, channel);
}

// Callback Manager

static uint8_t idx;
static void (*callback[BSP_TIM_LIMIT])();
static TIM_HandleTypeDef *handle[BSP_TIM_LIMIT];

void bsp_tim_callback(TIM_HandleTypeDef *h) {
    for(uint8_t i = 0; i < idx; i++) {
        if(h == handle[i] && callback[i]) {
            callback[i]();
        }
    }
}

void bsp_tim_it_enable(TIM_HandleTypeDef *h, void (*f)()) {
    handle[idx] = h, callback[idx] = f;
    HAL_TIM_Base_Start_IT(h);
    ++ idx;
}
