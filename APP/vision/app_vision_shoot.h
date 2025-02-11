//
// Created by fish on 2025/1/18.
//

#pragma once

#include <tuple>

#include "app_vision.h"

namespace shoot {
    struct armor {
        float x, y, z;          // 装甲板在世界坐标系的坐标
        float yaw;              // 装甲板坐标系相对世界坐标系的 yaw 角
    };

    struct shoot_params {
        float k;                // 弹道系数

        // 自身参数
        float current_v;        // 当前弹速
        float gimbal_pitch;     // 云台 pitch
        float gimbal_yaw;       // 云台 yaw

        // 目标参数
        float xw, yw, zw;       // ROS 坐标系下的 x, y, z
        float vxw, vyw, vzw;    // ROS 坐标系下的 vx, vy, vz
        float target_yaw;       // 目标 yaw
        float target_v_yaw;     // 目标 yaw 速度
        float r1;               // 目标中心到前后装甲板的距离
        float r2;               // 目标中心到左右装甲板的距离
        float dz;               // 另一对装甲板相对于被跟踪装甲板的高度差
        int bias_time;          // 偏置时间
        float s_bias;           // 枪口前推的距离
        float z_bias;           // yaw 轴电机到枪口水平面的垂直距离
    };

    // 单方向空气阻力模型
    float monoDirectionalAirResistanceModel(float s, float v, float angle);

    // pitch 弹道补偿
    float pitchTrajectoryCompensation(float s, float z, float v);

    // 计算结果
    std::tuple <float, float> solve(vision::RecvPacket *aim, float yaw, float pitch);
}