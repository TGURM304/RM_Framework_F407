//
// Created by fish on 2025/4/17.
//

#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

void bsp_key_set_callback(uint16_t GPIO_Pin, void (*f)(uint16_t GPIO_Pin));

#ifdef __cplusplus
}
#endif

#endif //BSP_KEY_H
