//
// Created by fish on 2024/12/18.
//

#include "app_vision.h"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <tuple>

#include "alg_crc.h"
#include "app_ins.h"
#include "bsp_def.h"
#include "bsp_time.h"
#include "bsp_uart.h"
#include "app_vision_shoot.h"

using namespace vision;

static RecvPacket rx_packet;
static VisionData vision_data;
static auto ins = app_ins_data();

std::tuple<float, float, float> calc_pitch_yaw_dist(float x, float y, float z) {
    float n = std::sqrt(x * x + y * y + z * z);
    return std::make_tuple(std::atan(z / std::sqrt(x * x + y * y)), std::atan(y / x), n);
}

float x, y, z;

void uart_rx_callback(bsp_uart_e e, uint8_t *s, uint16_t l) {
    if(l < sizeof rx_packet) return;
    if(!CRC16::verify(s, l)) return;
    std::copy_n(s, sizeof rx_packet, reinterpret_cast <uint8_t *> (&rx_packet));
    if(std::abs(rx_packet.x) > 1e4 |
       std::abs(rx_packet.y) > 1e4 |
       std::abs(rx_packet.z) > 1e4 |
       std::abs(rx_packet.r1) > 1e4 |
       std::abs(rx_packet.r2) > 1e4) return;
    if(rx_packet.tracking) {
        // x = rx_packet.x - rx_packet.r1 * std::cos(rx_packet.yaw);
        // y = rx_packet.y - rx_packet.r1 * std::sin(rx_packet.yaw);
        // z = rx_packet.z;
        // std::tie(vision_data.pitch, vision_data.yaw, vision_data.distance) = calc_pitch_yaw_dist(
        //     x, y, z - 0.05
        // );

        std::tie(vision_data.pitch, vision_data.yaw) = shoot::solve(
            &rx_packet, ins->yaw / 180 * M_PI, ins->roll / 180 * M_PI
        );

        if(std::isnan(vision_data.yaw) or std::isnan(vision_data.pitch))
            while(true) __NOP();

        vision_data.yaw *= 180 / M_PI;
        vision_data.pitch *= 180 / M_PI;
    } else {
        vision_data.yaw = vision_data.pitch = 0;
    }
    vision_data.activate = rx_packet.tracking;
    vision_data.recv_count ++;
    vision_data.time_stamp = bsp_time_get_ms();
}

void vision::init() {
    bsp_uart_set_callback(E_UART_VISION, uart_rx_callback);
}

RecvPacket *vision::recv() {
    return &rx_packet;
}

VisionData *vision::data() {
    return &vision_data;
}

void vision::send(uint8_t detect_color, bool reset_tracker) {
    SendPacket pkg = {
        .detect_color = detect_color,
        .reset_tracker = reset_tracker,
        .reserved = 0,
        /* 世界坐标系下云台姿态  */
        .roll = static_cast <float> (ins->pitch / 180 * M_PI),
        .pitch = static_cast <float> (-ins->roll / 180 * M_PI),
        .yaw = static_cast <float> (ins->yaw / 180 * M_PI),
        /* 当前云台瞄准的位置，用于发布可视化 Marker */
        .aim_x = static_cast <float> (vision_data.distance * std::cos( ins->yaw / 180 * M_PI)),
        .aim_y = static_cast <float> (vision_data.distance * std::sin( ins->yaw / 180 * M_PI)),
        .aim_z = - static_cast <float> (vision_data.distance * std::sin(-ins->roll / 180 * M_PI) - 0.05),
        .checksum = 0
    };

    CRC16::append(pkg);

    bsp_uart_send(E_UART_VISION, reinterpret_cast <uint8_t *> (&pkg), sizeof pkg);
}