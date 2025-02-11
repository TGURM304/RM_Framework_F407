//
// Created by fish on 2025/1/18.
//

#include "app_vision_shoot.h"
#include "app_conf.h"

#include <array>
#include <cmath>
#include <vector>

#include "bsp_def.h"

using namespace shoot;

armor armors[4];

// 重力加速度
const float g = 9.81;

// 飞行时间
static float t = 0.5;

// 小弹丸弹道系数
static float k = 0.038;

// 偏置时间
static float bias_time = 0;

// 枪口前推的距离
static float s_bias = 0;

// 电机到枪口水平面的垂直距离
static float z_bias = -0.05;

// 射速 (m/s)
static float shoot_vel = 15;

/*!
 * 单方向空气阻力弹道模型
 * @param s 距离 (m)
 * @param v 速度 (m/s)
 * @param angle 角度 (rad)
 * @return 高度 (m)
 */
float shoot::monoDirectionalAirResistanceModel(float s, float v, float angle) {
    t = (std::exp(k * s) - 1) / (k * v * std::cos(angle));
    if(std::isnan(t)) while(true) __NOP();
    if(t < 0) {
        return t = 0;
    }
    return v * t * std::sin(angle) - g * t * t / 2;
}

/*!
 * 计算 pitch 角
 * @param s 距离 (m)
 * @param z 高度 (m)
 * @param v 速度 (m/s)
 * @return angle_pitch (rad)
 */
float shoot::pitchTrajectoryCompensation(float s, float z, float v){
    float z_temp = z, z_actual = 0, dz = 0, angle_pitch = 0;
    if(z_temp == 0) z_temp += 0.00001;
    for(int i = 0; i < 20; i++) {
        angle_pitch = std::atan2(z_temp, s);
        z_actual = monoDirectionalAirResistanceModel(s, v, angle_pitch);
        if(z_actual == 0) {
            angle_pitch = 0;
            break;
        }
        dz = 0.3f * (z - z_actual);
        z_temp += dz;
        if(std::abs(dz) < 0.00001)
            break;
    }
    return angle_pitch;
}

std::tuple<float, float> shoot::solve(vision::RecvPacket *aim, float yaw, float pitch){
    // 线性预测，延迟时间 = 偏置时间 +飞行时间
    float delay = bias_time + t, target_yaw = aim->yaw + aim->v_yaw * delay;

    // 计算四块装甲板的位置
    // 装甲板 id 顺序，逆时针编号
    //       2
    //    3     1
    //       0

    // 选择的装甲板
    int selected_idx = 0;

    for(int i = 0; i < 4; i++) {
        auto cur_yaw = static_cast <float> (target_yaw + i * M_PI_2);
        auto r = (i & 1) ? aim->r2 : aim->r1;
        armors[i].yaw = cur_yaw;
        armors[i].x = aim->x - r * std::cos(cur_yaw);
        armors[i].y = aim->y - r * std::sin(cur_yaw);
        armors[i].z = (i & 1) ? aim->z + aim->dz : aim->z;
    }

    // float mn = std::abs(armors[0].yaw - yaw);
    // for(int i = 1; i < 4; i++) {
    //     if(std::abs(armors[i].yaw - yaw) < mn) {
    //         mn = std::abs(armors[i].yaw - yaw);
    //         selected_idx = i;
    //     }
    // }

    auto [aim_x, aim_y, aim_z] = std::array {
        armors[selected_idx].x + aim->vx * delay,
        armors[selected_idx].y + aim->vy * delay,
        armors[selected_idx].z + aim->vz * delay,
    };

    float temp_pitch = -pitchTrajectoryCompensation(
        std::sqrt(aim_x * aim_x + aim_y * aim_y - s_bias),
        aim_z + z_bias,
        shoot_vel
    );

    return { temp_pitch != 0 ? -temp_pitch : pitch, aim_x != 0 || aim_y != 0 ? std::atan2(aim_y, aim_x) : yaw };
}
