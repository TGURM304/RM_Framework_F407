//
// Created by fish on 2025/2/7.
//

#pragma once
#include "bsp_sys.h"
#include "message_buffer.h"
#include "portmacro.h"
#include "projdefs.h"

#include <type_traits>

namespace OS {
class Message {
public:
    explicit Message(size_t length) : msg_(xMessageBufferCreate(length)) {}

    template <typename T>
    bool send(const T& data, size_t sz = sizeof(T)) {
        if(bsp_sys_in_isr()) {
            BaseType_t xHigherPriorityTaskWoken;
            auto re = xMessageBufferSendFromISR(msg_, &data, sz, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return re == pdTRUE;
        } else {
            return xMessageBufferSend(msg_, &data, 0, sz) == pdTRUE;
        }
    }

    template <typename T>
    bool receive(T& data, size_t sz = sizeof(T)) {
        if(bsp_sys_in_isr()) {
            BaseType_t xHigherPriorityTaskWoken;
            auto re = xMessageBufferReceiveFromISR(msg_, &data, sz, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
            return re == pdTRUE;
        } else {
            return xMessageBufferReceive(msg_, &data, sz, 0) == pdTRUE;
        }
    }

    bool reset() { return xMessageBufferReset(msg_) == pdTRUE; }

    bool empty() {
        return xMessageBufferIsEmpty(msg_);
    }
private:
    MessageBufferHandle_t msg_ = nullptr;
};
}
