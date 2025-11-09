#include "stm32f10x.h"                  
#include "Delay.h"
#include "OLED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"
#include "Encoder.h"
#include "Serial.h"
#include "string.h"
#include <math.h>
#include <stdint.h>
#include <stdio.h>

int16_t ExtractSpeed(void);
void Speed_control(void);
void Motor_drive(void);

/* 电机测试变量 */
uint8_t KeyNum;
int16_t PWM;
uint8_t mode = 1;                       // 0: Motor drive, 1: Speed control

/* 速度控制模式PID参数 */
float Kp = 5.0f, Ki = 0.5f, Kd = 0.3f;

/* 位置保持模式参数 - 实现弹簧效应 */
#define POSITION_KP 15.0f               // 位置比例系数，决定回正力强度
#define POSITION_KD 2.0f                // 位置微分系数，抑制震荡
#define MAX_RETURN_FORCE 80             // 最大回正力输出

/* 控制变量定义 */
float Target = 0, Actual = 0, Out = 0;           
float Actual_left = 0;
float Target_right = 0, Actual_right = 0, Out_right = 0;  

// 速度控制模式误差
float Error0_left = 0, Error1_left = 0, ErrorInt_left = 0;           
float Error0_right = 0, Error1_right = 0, ErrorInt_right = 0;     

// 位置保持相关变量
static int32_t Left_Home_Position = 0;    // 左电机原点位置
static int32_t Right_Home_Position = 0;   // 右电机原点位置  
static int32_t Left_Current_Position = 0; // 左电机当前位置
static int32_t Right_Current_Position = 0;// 右电机当前位置
static uint8_t position_hold_enabled = 0; // 位置保持使能标志

// 速度滤波变量
static float Filtered_Actual = 0, Filtered_Actual_right = 0;

int main(void)
{
    /* 模块初始化 */
    OLED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Serial_Init();
    Timer_Init();       // 10ms中断
    
    OLED_Printf(0, 0, OLED_8X16, "Init Complete!");
    OLED_Update();
    Delay_ms(1000);
    
    while (1)
    {
        KeyNum = Key_GetNum();
        
        if (KeyNum == 1)
        {
            mode = !mode;
            
            if (mode == 0)  // Motor drive模式
            {
                position_hold_enabled = 0;
                Actual_left = 0;
                Actual_right = 0;
                Target_right = 0;
                Error0_right = 0;
                Error1_right = 0;
                ErrorInt_right = 0;
                Left_Current_Position = 0;
                Right_Current_Position = 0;
                
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
                OLED_Clear();
                OLED_Printf(0, 0, OLED_8X16, "Mode: Motor Drive");
            }
            else  // Speed control模式
            {
                // 在切换到速度控制模式时设置当前位置为原点
                Left_Home_Position = Left_Current_Position;
                Right_Home_Position = Right_Current_Position;
                position_hold_enabled = 1;
                
                Target = 0;
                Actual = 0;
                Out = 0;
                Error0_left = 0;
                Error1_left = 0;
                ErrorInt_left = 0;
                Actual_right = 0;
                Out_right = 0;
                Error0_right = 0;
                Error1_right = 0;
                ErrorInt_right = 0;
                Filtered_Actual = 0;
                Filtered_Actual_right = 0;
                
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
                OLED_Clear();
                OLED_Printf(0, 0, OLED_8X16, "Mode: Speed Control");
                OLED_Printf(0, 16, OLED_8X16, "Position Hold: ON");
            }
            OLED_Update();
            Delay_ms(200);
        }
        
        if (mode == 0)
        {
            Motor_drive();
        }
        else
        {
            Speed_control();
        }
        
        OLED_Update();
        Delay_ms(10);
    }
}

/* 10ms定时器中断 - 实现位置保持的弹簧效应 */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (mode == 1)  // Speed control模式
        {            
            /* 1. 读取左右编码器速度并更新位置 */
            int16_t raw_left = Encoder_Get_Left();
            int16_t raw_right = Encoder_Get_Right();
            Encoder_Clear_Both();
            
            // 更新当前位置（累积编码器值）
            Left_Current_Position += raw_left;
            Right_Current_Position += raw_right;
            
            /* 2. 速度滤波 */
            Filtered_Actual = 0.6f * Filtered_Actual + 0.4f * raw_left;
            Filtered_Actual_right = 0.6f * Filtered_Actual_right + 0.4f * raw_right;
            
            Actual = Filtered_Actual;
            Actual_right = Filtered_Actual_right;
            
            /* 3. 判断控制模式：速度跟踪 or 位置保持 */
            if (Target != 0) 
            {
                // 速度跟踪模式 - 使用原有速度PID
                position_hold_enabled = 0;
                
                Error1_left = Error0_left;
                Error0_left = Target - Actual;
                ErrorInt_left += Error0_left;
                
                if (ErrorInt_left > 100) ErrorInt_left = 100;
                if (ErrorInt_left < -100) ErrorInt_left = -100;
                
                Out = Kp * Error0_left + Ki * ErrorInt_left + Kd * (Error0_left - Error1_left);
                if (Out > 80) Out = 80;
                if (Out < -80) Out = -80;
                
                Error1_right = Error0_right;
                Error0_right = Target - Actual_right;
                ErrorInt_right += Error0_right;
                
                if (ErrorInt_right > 100) ErrorInt_right = 100;
                if (ErrorInt_right < -100) ErrorInt_right = -100;
                
                Out_right = Kp * Error0_right + Ki * ErrorInt_right + Kd * (Error0_right - Error1_right);
                if (Out_right > 80) Out_right = 80;
                if (Out_right < -80) Out_right = -80;
            }
            else 
            {
                // 位置保持模式 - 实现弹簧效应
                if (!position_hold_enabled) 
                {
                    // 当目标速度第一次变为0时，设置当前位置为原点
                    Left_Home_Position = Left_Current_Position;
                    Right_Home_Position = Right_Current_Position;
                    position_hold_enabled = 1;
                }
                
                /* 左电机位置控制 - 弹簧效应 */
                int32_t left_position_error = Left_Home_Position - Left_Current_Position;
                
                // 计算位置变化率（近似速度）
                float left_velocity = Actual;
                
                // 位置PD控制：P项提供回正力，D项抑制震荡
                Out = POSITION_KP * left_position_error * 0.01f - POSITION_KD * left_velocity;
                
                // 非线性增强：位置误差越大，回正力越强
                float force_boost = 1.0f + fabs(left_position_error) * 0.005f;
                Out *= force_boost;
                
                // 输出限幅
                if (Out > MAX_RETURN_FORCE) Out = MAX_RETURN_FORCE;
                if (Out < -MAX_RETURN_FORCE) Out = -MAX_RETURN_FORCE;
                
                /* 右电机位置控制 - 弹簧效应 */
                int32_t right_position_error = Right_Home_Position - Right_Current_Position;
                float right_velocity = Actual_right;
                
                Out_right = POSITION_KP * right_position_error * 0.01f - POSITION_KD * right_velocity;
                Out_right *= (1.0f + fabs(right_position_error) * 0.005f);
                
                if (Out_right > MAX_RETURN_FORCE) Out_right = MAX_RETURN_FORCE;
                if (Out_right < -MAX_RETURN_FORCE) Out_right = -MAX_RETURN_FORCE;
                
                // 小误差死区处理，防止微震
                if (fabs(left_position_error) < 5 && fabs(left_velocity) < 2) {
                    Out = 0;
                }
                if (fabs(right_position_error) < 5 && fabs(right_velocity) < 2) {
                    Out_right = 0;
                }
            }
            
            /* 4. 执行控制 */
            Motor_SetPWM((int8_t)Out);
            Motor_SetPWM_right((int8_t)Out_right);
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

/* 速度控制模式 - 增强显示位置信息 */
void Speed_control(void)
{
    Target = ExtractSpeed();
    
    /* OLED显示 */
    if (Target == 0 && position_hold_enabled) 
    {
        // 位置保持模式显示
        int32_t left_pos_error = Left_Home_Position - Left_Current_Position;
        int32_t right_pos_error = Right_Home_Position - Right_Current_Position;
        
        OLED_Printf(0, 16, OLED_8X16, "PosErr:L%+05ld", left_pos_error);
        OLED_Printf(0, 32, OLED_8X16, "PosErr:R%+05ld", right_pos_error);
        OLED_Printf(0, 48, OLED_8X16, "Force:L%+03.0f R%+03.0f", Out, Out_right);
        
        OLED_Printf(64, 16, OLED_8X16, "Mode:Pos Hold");
        OLED_Printf(64, 32, OLED_8X16, "Kp:%4.1f", POSITION_KP);
        OLED_Printf(64, 48, OLED_8X16, "Kd:%4.1f", POSITION_KD);
    }
    else 
    {
        // 速度跟踪模式显示
        OLED_Printf(0, 16, OLED_8X16, "L:%+04.0f R:%+04.0f", Actual, Actual_right);
        OLED_Printf(0, 32, OLED_8X16, "Tar:%+04.0f", Target);
        OLED_Printf(0, 48, OLED_8X16, "Out:L%+03.0f R%+03.0f", Out, Out_right);
        
        OLED_Printf(64, 16, OLED_8X16, "Mode:Speed Track");
        OLED_Printf(64, 32, OLED_8X16, "Kp:%4.1f", Kp);
        OLED_Printf(64, 48, OLED_8X16, "Kd:%4.1f", Kd);
    }
    
    /* 串口输出 */
    static uint32_t Tick = 0;
    if (Tick++ % 5 == 0)
    {
        if (Target == 0 && position_hold_enabled) 
        {
            int32_t left_pos_error = Left_Home_Position - Left_Current_Position;
            int32_t right_pos_error = Right_Home_Position - Right_Current_Position;
            Serial_Printf("PositionHold, L_Err:%ld, R_Err:%ld, L_Force:%.0f, R_Force:%.0f\n", 
                         left_pos_error, right_pos_error, Out, Out_right);
        }
        else 
        {
            Serial_Printf("%.0f,%.0f,%.0f\n", 
                         Target, Actual, Actual_right);
        }
    }
}

/* 电机驱动模式 - 保持不变 */
void Motor_drive(void)
{
    int16_t left_delta = Encoder_Get_Left();
    int16_t right_delta = Encoder_Get_Right();
    Encoder_Clear_Both();
    
    // 更新位置（用于显示）
    Left_Current_Position += left_delta;
    Right_Current_Position += right_delta;
    
    Actual_left = Left_Current_Position;
    Actual_right = Right_Current_Position;
    
    Target_right = Actual_left;
    
    // 位置式PID
    Error1_right = Error0_right;
    Error0_right = Target_right - Actual_right;
    
    // 积分分离
    if (fabs(Error0_right) < 80)
    {
        ErrorInt_right += Error0_right;
        ErrorInt_right = (ErrorInt_right > 200) ? 200 :
                        ((ErrorInt_right < -200) ? -200 : ErrorInt_right);
    }
    else
    {
        ErrorInt_right *= 0.9f;
    }
    
    // 位置PID计算
    Out_right = 0.8f * Error0_right + 
                0.015f * ErrorInt_right + 
                0.25f * (Error0_right - Error1_right);
    
    // 输出限幅
    if (Out_right > 80) Out_right = 80;
    if (Out_right < -80) Out_right = -80;
    
    // 跟随模式死区补偿
    if (fabs(Out_right) < 8 && fabs(Error0_right) > 15)
    {
        Out_right = (Out_right > 0) ? 8 : -8;
    }
    
    Motor_SetPWM_right((int8_t)Out_right);
    
    // OLED显示
    OLED_Printf(0, 16, OLED_8X16, "L_Pos:%+06.0f", Actual_left);
    OLED_Printf(0, 32, OLED_8X16, "R_Pos:%+06.0f", Actual_right);
    OLED_Printf(0, 48, OLED_8X16, "Out:%+03.0f Err:%+04.0f", Out_right, Error0_right);
    
    // 串口输出
    static uint32_t follow_tick = 0;
    if (follow_tick++ % 5 == 0)
    {
        Serial_Printf("L=%.0f,R_Tar=%.0f,R=%.0f,Err=%.0f\n", 
                      Actual_left, Target_right, Actual_right, Error0_right);
    }
}

/* 串口指令解析 - 保持不变 */
int16_t ExtractSpeed(void)
{
    static int16_t last = 0;
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        
        char *cmd_prefix = strstr(Serial_RxPacket, "@speed%");
        if (cmd_prefix != NULL)
        {
            cmd_prefix += 7;
            
            // 跳过空格
            while (*cmd_prefix == ' ' && *cmd_prefix != '\0') cmd_prefix++;
            
            // 解析符号
            int8_t sign = 1;
            if (*cmd_prefix == '-')
            {
                sign = -1;
                cmd_prefix++;
            }
            else if (*cmd_prefix == '+')
            {
                cmd_prefix++;
            }
            
            // 解析数字
            uint16_t speed = 0;
            uint8_t has_digit = 0;
            while (*cmd_prefix >= '0' && *cmd_prefix <= '9' && *cmd_prefix != '\0')
            {
                speed = speed * 10 + (*cmd_prefix - '0');
                cmd_prefix++;
                has_digit = 1;
            }
            
            if (has_digit)
            {
                last = sign * (int16_t)speed;
                if (last > 100) last = 100;
                if (last < -100) last = -100;
                
                // 当设置非零速度时，禁用位置保持
                if (last != 0) {
                    position_hold_enabled = 0;
                }
                
                Serial_Printf("Set Speed: %d\n", last);
            }
        }
    }
    return last;
}