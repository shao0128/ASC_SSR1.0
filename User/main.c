#include "stm32f10x.h"                  
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

/* 控制变量定义 */
uint8_t KeyNum;
int16_t TargetSpeed = 0;               // 目标速度
uint8_t car_enabled = 0;               // 小车使能标志
int16_t base_speed = 30;               // 基础速度

/* 编码器相关变量 */
int16_t Actual_left = 0, Actual_right = 0;
int32_t Left_Current_Position = 0, Right_Current_Position = 0;

void Car_Control(void);
void Update_Display(void);

int main(void)
{
    /* 模块初始化 */
    OLED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Timer_Init();       // 10ms中断
    
    OLED_Printf(0, 0, OLED_8X16, "Car Control System");
    OLED_Printf(0, 16, OLED_8X16, "Press B10 to Start");
    OLED_Printf(0, 32, OLED_8X16, "B11:Speed +40");
    OLED_Update();
    Delay_ms(2000);
    
    OLED_Clear();
    
    while (1)
    {
        KeyNum = Key_GetNum();
        
        /* 按键1（B10）：启动/停止小车 */
        if (KeyNum == 1)
        {
            car_enabled = !car_enabled;
            if (car_enabled)
            {
                TargetSpeed = base_speed;
                OLED_Printf(0, 0, OLED_8X16, "Status: Running ");
            }
            else
            {
                TargetSpeed = 0;
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
                OLED_Printf(0, 0, OLED_8X16, "Status: Stopped ");
            }
        }
        
        /* 按键2（B11）：速度增加40 */
        if (KeyNum == 2 && car_enabled)
        {
            base_speed += 40;
            if (base_speed > 100) base_speed = 100;
            TargetSpeed = base_speed;
        }
        
        Car_Control();
        Update_Display();
        
        OLED_Update();
        Delay_ms(10);
    }
}

/* 10ms定时器中断 - 电机控制 */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (car_enabled)
        {
            /* 读取编码器值并清零 */
            int16_t left_delta = Encoder_Get_Left();
            int16_t right_delta = Encoder_Get_Right();
            Encoder_Clear_Both();
            
            /* 更新位置 */
            Left_Current_Position += left_delta;
            Right_Current_Position += right_delta;
            
            /* 设置电机速度 */
            Motor_SetPWM(TargetSpeed);
            Motor_SetPWM_right(TargetSpeed);
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

/* 小车控制函数 */
void Car_Control(void)
{
    // 主要控制逻辑在定时器中断中处理
    // 这里可以添加其他控制逻辑
}

/* 更新显示 */
void Update_Display(void)
{
    OLED_Printf(0, 16, OLED_8X16, "BaseSpeed: %3d", base_speed);
    OLED_Printf(0, 32, OLED_8X16, "Target:    %3d", TargetSpeed);
    OLED_Printf(0, 48, OLED_8X16, "L_Pos:%+06ld", Left_Current_Position);
    
    OLED_Printf(64, 16, OLED_8X16, "R_Pos:%+06ld", Right_Current_Position);
    OLED_Printf(64, 32, OLED_8X16, "B10:Start/Stop");
    OLED_Printf(64, 48, OLED_8X16, "B11:Speed +40");
}