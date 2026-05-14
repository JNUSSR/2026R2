//
// Created by chengfeng on 2026/5/12.
//

#include "alg_leso.h"

void FirstOrderSystemESO::Init(float f, float b, float y, float kp, float ts) {
    if (!InitFlag) {
        float omega = 2 * PI * f;
        l1 = 2 * omega;
        l2 = omega * omega;

        this->b = b;

        x1 = y;
        x2 = 0.0f;

        Kp = kp;

        Ts = ts;
        InitFlag = true;
    }
}

void FirstOrderSystemESO::Update(float y, float u)
{
    // 基础的比例控制
    float u0 = Kp * (Target - x1);
    // 计算观测误差
    float e = y - x1;
    // 更新状态变量
    float x1_next = x1 + (x2 + l1 * e + b * u) * Ts;
    // 扰动估计
    float x2_next = x2 + l2 * e * Ts;
    // 计算补偿后的系统输出
    out = (u0 - x2_next) / b;

    x1 = x1_next;
    x2 = x2_next;
}

// float FirstOrderSystemESO::Get_f(float y,float u) {
//     float temp = x2;
//     Update(y,u);
//     return temp;
// }

float FirstOrderSystemESO::Get_b() {
    return b;
}

float FirstOrderSystemESO::Get_x1() {
    return x1;
}

float FirstOrderSystemESO::Get_x2() {
    return x2;
}

float FirstOrderSystemESO::Get_Out() {
    return out;
}

/**
 * @brief 重置观测器状态
 * @param y 重置的系统输出
 */
void FirstOrderSystemESO::Reset(float y) {
    x1 = y;
    x2 = 0.0f;
}

void FirstOrderSystemESO::Set_Target(float target) {
    Target = target;
}
