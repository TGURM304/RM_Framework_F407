//
// Created by fish on 2024/10/6.
//

#include "bsp_adc.h"

#include "adc.h"
#include "bsp_tim.h"

#include "stdint.h"

/*
 *  bsp_adc
 *  读 vbus 电压用
 *  TODO: 精度待验证
 */

static uint16_t val[2];

void bsp_adc_init(void) {
    bsp_tim_config(&htim3, 500);
    HAL_TIM_Base_Start(&htim3);
    HAL_ADC_Start_DMA(&hadc3, (uint32_t *) val, 2);
}

float bsp_adc_vbus(void) {
    return 3.3f * (float) val[0] * 222 / 4095 / 22;
}