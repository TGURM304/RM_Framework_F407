//
// Created by fish on 2025/4/17.
//

#include "bsp_key.h"

#include "bsp_def.h"
#include "bsp_time.h"

static void (*callback[16])(uint16_t GPIO_Pin);
static uint32_t last_interrupt_time[16];

void bsp_key_set_callback(uint16_t GPIO_Pin, void (*f)(uint16_t GPIO_Pin)) {
    for(uint8_t i = 0; i < 16; i++) {
        if(GPIO_Pin >> i & 1) {
            BSP_ASSERT(callback[i] == NULL);
            callback[i] = f;
        }
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    for(uint8_t i = 0; i < 16; i++) {
        if(GPIO_Pin >> i & 1 && callback[i] != NULL) {
            if(bsp_time_get_ms() - last_interrupt_time[i] > 100) callback[i](GPIO_Pin);
            last_interrupt_time[i] = bsp_time_get_ms();
        }
    }
}