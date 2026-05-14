//
// Created by chengfeng on 2026/5/12.
//

#ifndef ALG_LESO_H
#define ALG_LESO_H

#include "drv_math.h"

class FirstOrderSystemESO {
public:
    void Init(float f, float b, float y, float kp, float ts = 0.001f);
    
    void Update(float y, float u);
    //float Get_f(float y,float u);
    float Get_b();
    float Get_x1();
    float Get_x2();
    float Get_Out();
    void Reset(float y = 0.0f);
    void Set_Target(float target);
private:
    bool InitFlag = false;
    // 反馈补偿增益
    float l1 = 0.0f;
    // 扰动补偿增益
    float l2 = 0.0f;
    // 采样时间
    float Ts = 0.001f;
    // 状态变量, 估计的系统输出
    float x1 = 0.0f;
    // 扰动估计
    float x2 = 0.0f;
    // 系统输出的目标值
    float Target = 0.0f;
    // 补偿后的系统输出
    float out = 0.0f;
    // 比例增益
    float Kp = 0.0f;
    // 输入增益
    float b = 0.0f;

};

#endif // ALG_LESO_H
