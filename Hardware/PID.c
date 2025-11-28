#include "stm32f10x.h"                  // Device header
#include "Motor.h"
#include "PID.h"

// 电机目标速度
extern int16_t Speed_Target;  // 30,60,90

// 电机实际速度
extern int16_t Speed1;
extern int16_t Speed2;

// 共用PID参数，便于统一调整
static float kp = 1.0f, ki = 1.5f, kd = 0.05f;

// 通用PID计算函数
static float PID_Calculate(float target, float actual, 
                          float *error0, float *error1, float *error2)
{
    float output = 0;
    
    *error2 = *error1;
    *error1 = *error0;
    *error0 = target - actual;
    
    // 增量式PID
    output += kp * (*error0 - *error1) + ki * (*error0) + kd * (*error0 - 2 * (*error1) + *error2);
    
    // 输出限幅
    if (output > 100) output = 100;
    if (output < 0) output = 0;
    
    return output;
}

// 红外传感器判断小车应走直线
void PID_Straight(void)
{
    static float error0_1 = 0, error1_1 = 0, error2_1 = 0;
    static float error0_2 = 0, error1_2 = 0, error2_2 = 0;
    
    float Out1 = PID_Calculate((float)Speed_Target, (float)Speed1, &error0_1, &error1_1, &error2_1);
    float Out2 = PID_Calculate((float)Speed_Target, (float)Speed2, &error0_2, &error1_2, &error2_2);
    
    Moter_SetPower1((int8_t)(-Out1));
    Moter_SetPower2((int8_t)(-Out2));
}

// 红外传感器判断小车应向右轻微拐弯
void PID_RightSlight(void)
{
    static float error0_1 = 0, error1_1 = 0, error2_1 = 0;
    static float error0_2 = 0, error1_2 = 0, error2_2 = 0;
    
    // 轻微右转：左轮保持速度，右轮减速
    float Out1 = PID_Calculate((float)Speed_Target, (float)Speed1, &error0_1, &error1_1, &error2_1);
    float Out2 = PID_Calculate((float)Speed_Target * 0.7f, (float)Speed2, &error0_2, &error1_2, &error2_2); // 右轮速度降低30%
    
    Moter_SetPower1((int8_t)(-Out1));
    Moter_SetPower2((int8_t)(-Out2));
}

// 红外传感器判断小车应向左轻微拐弯
void PID_LeftSlight(void)
{
    static float error0_1 = 0, error1_1 = 0, error2_1 = 0;
    static float error0_2 = 0, error1_2 = 0, error2_2 = 0;
    
    // 轻微左转：右轮保持速度，左轮减速
    float Out1 = PID_Calculate((float)Speed_Target * 0.7f, (float)Speed1, &error0_1, &error1_1, &error2_1); // 左轮速度降低30%
    float Out2 = PID_Calculate((float)Speed_Target, (float)Speed2, &error0_2, &error1_2, &error2_2);
    
    Moter_SetPower1((int8_t)(-Out1));
    Moter_SetPower2((int8_t)(-Out2));
}

// 红外传感器判断小车应向右直角拐弯
void PID_RightStraight(void)
{
    static float error0_1 = 0, error1_1 = 0, error2_1 = 0;
    static float error0_2 = 0, error1_2 = 0, error2_2 = 0;
    
    // 直角右转：左轮正转，右轮反转形成旋转
    float Out1 = PID_Calculate((float)Speed_Target, (float)Speed1, &error0_1, &error1_1, &error2_1);
    float Out2 = PID_Calculate((float)(-Speed_Target) * 0.5f, (float)Speed2, &error0_2, &error1_2, &error2_2); // 右轮反转
    
    Moter_SetPower1((int8_t)(-Out1));
    Moter_SetPower2((int8_t)(-Out2));
}

// 红外传感器判断小车应向左直角拐弯
void PID_LeftStraight(void)
{
    static float error0_1 = 0, error1_1 = 0, error2_1 = 0;
    static float error0_2 = 0, error1_2 = 0, error2_2 = 0;
    
    // 直角左转：右轮正转，左轮反转形成旋转
    float Out1 = PID_Calculate((float)(-Speed_Target) * 0.5f, (float)Speed1, &error0_1, &error1_1, &error2_1); // 左轮反转
    float Out2 = PID_Calculate((float)Speed_Target, (float)Speed2, &error0_2, &error1_2, &error2_2);
    
    Moter_SetPower1((int8_t)(-Out1));
    Moter_SetPower2((int8_t)(-Out2));
}

void PID_Still(void)
{
    Moter_SetPower1(0);
    Moter_SetPower2(0);
}