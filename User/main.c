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

int16_t ExtractSpeed(void);
void Speed_control(void);
void Motor_drive(void);

/* 电机测试变量 */
uint8_t KeyNum;
int16_t PWM;
uint8_t mode = 1;                       // 0: Motor drive, 1: Speed control

/* 速度控制模式PID参数（优化消除抖动） */
#define KP_SPEED 1.2f      // 比例响应
#define KI_SPEED 0.08f     // 积分
#define KD_SPEED 0.15f     // 微分
#define INTEGRAL_LIMIT 25  // 积分限幅

/* 回正力参数（优化消除抖动） */
#define KP_POS_RETURN 1.2f  // 降低回正力比例，减少抖动
#define POS_LIMIT 35       // 降低回正输出限幅
#define POS_DEAD_ZONE 3    // 位置死区，小位置偏差时不输出

/* 跟随模式PID参数 */
#define KP_FOLLOW 0.8f     
#define KI_FOLLOW 0.015f   
#define KD_FOLLOW 0.25f    
#define FOLLOW_INTEGRAL_LIMIT 200

/* 控制变量定义 */
float Target = 0, Actual = 0, Out = 0;           
float Actual_left = 0;
float Target_right = 0, Actual_right = 0, Out_right = 0;  

// 速度控制模式误差
float Error0 = 0, Error1 = 0, Error2 = 0;           
float Error0_r = 0, Error1_r = 0, Error2_r = 0;     

// 跟随模式误差
float Error0_right = 0, Error1_right = 0, ErrorInt_right = 0;  

static float Integral = 0, Integral_r = 0;          
static int32_t Left_Pos = 0, Right_Pos = 0;         

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
                Actual_left = 0;
                Actual_right = 0;
                Target_right = 0;
                Error0_right = 0;
                Error1_right = 0;
                ErrorInt_right = 0;
                Left_Pos = 0;
                Right_Pos = 0;
                Integral = 0;
                Integral_r = 0;
                
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
                OLED_Clear();
                OLED_Printf(0, 0, OLED_8X16, "Mode: Motor Drive");
            }
            else  // Speed control模式
            {
                Target = 0;
                Actual = 0;
                Out = 0;
                Error0 = 0;
                Error1 = 0;
                Error2 = 0;
                Actual_right = 0;
                Out_right = 0;
                Error0_r = 0;
                Error1_r = 0;
                Error2_r = 0;
                Left_Pos = 0;
                Right_Pos = 0;
                Integral = 0;
                Integral_r = 0;
                Filtered_Actual = 0;
                Filtered_Actual_right = 0;
                
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
                OLED_Clear();
                OLED_Printf(0, 0, OLED_8X16, "Mode: Speed Control");
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

/* 10ms定时器中断：优化消除抖动 */
void TIM1_UP_IRQHandler(void)
{
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (mode == 1)  // Speed control模式
        {            
            /* 1. 读取编码器 */
            int16_t raw_left = Encoder_Get_Left();
            int16_t raw_right = Encoder_Get_Right();
            Encoder_Clear_Both();
            
            /* 2. 改进的速度滤波（更强的滤波减少噪声） */
            Filtered_Actual = 0.8f * Filtered_Actual + 0.2f * raw_left;
            Filtered_Actual_right = 0.8f * Filtered_Actual_right + 0.2f * raw_right;
            
            Actual = Filtered_Actual;
            Actual_right = Filtered_Actual_right;
            
            /* 3. 优化的回正力逻辑 - 消除抖动 */
            float Pos_Compensate = 0, Pos_Compensate_r = 0;
            if (Target == 0)
            {
                // 累计位置偏差（带衰减）
                Left_Pos = 0.9f * Left_Pos + raw_left;
                Right_Pos = 0.9f * Right_Pos + raw_right;
                
                // 添加位置死区 - 小偏差时不输出回正力
                if (fabs(Left_Pos) > POS_DEAD_ZONE)
                {
                    Pos_Compensate = -KP_POS_RETURN * Left_Pos;
                }
                if (fabs(Right_Pos) > POS_DEAD_ZONE)
                {
                    Pos_Compensate_r = -KP_POS_RETURN * Right_Pos;
                }
                
                // 回正输出限幅
                Pos_Compensate = (Pos_Compensate > POS_LIMIT) ? POS_LIMIT : 
                                ((Pos_Compensate < -POS_LIMIT) ? -POS_LIMIT : Pos_Compensate);
                Pos_Compensate_r = (Pos_Compensate_r > POS_LIMIT) ? POS_LIMIT : 
                                  ((Pos_Compensate_r < -POS_LIMIT) ? -POS_LIMIT : Pos_Compensate_r);
            }
            else
            {
                // 非零目标时，清零位置累积
                Left_Pos = 0;
                Right_Pos = 0;
            }
            
            /* 4. 左电机PID计算 */
            Error2 = Error1;
            Error1 = Error0;
            Error0 = Target - Actual;
            
            // 积分项（带限幅）
            Integral += Error0;
            Integral = (Integral > INTEGRAL_LIMIT) ? INTEGRAL_LIMIT : 
                      ((Integral < -INTEGRAL_LIMIT) ? -INTEGRAL_LIMIT : Integral);
            
            // 增量式PID
            float deltaOut = KP_SPEED * (Error0 - Error1) + 
                           KI_SPEED * Integral + 
                           KD_SPEED * (Error0 - 2*Error1 + Error2);
            Out += deltaOut;
            
            // 只在目标为0时添加回正补偿
            if (Target == 0)
            {
                Out += Pos_Compensate;
            }
            
            /* 5. 右电机PID计算 */
            Error2_r = Error1_r;
            Error1_r = Error0_r;
            Error0_r = Target - Actual_right;
            
            Integral_r += Error0_r;
            Integral_r = (Integral_r > INTEGRAL_LIMIT) ? INTEGRAL_LIMIT : 
                        ((Integral_r < -INTEGRAL_LIMIT) ? -INTEGRAL_LIMIT : Integral_r);
            
            float deltaOut_r = KP_SPEED * (Error0_r - Error1_r) + 
                             KI_SPEED * Integral_r + 
                             KD_SPEED * (Error0_r - 2*Error1_r + Error2_r);
            Out_right += deltaOut_r;
            
            if (Target == 0)
            {
                Out_right += Pos_Compensate_r;
            }
            
            /* 6. 关键优化：输出处理 - 完全消除抖动 */
            // 对称输出限幅
            if (Out > 100) Out = 100;
            if (Out < -100) Out = -100;
            if (Out_right > 100) Out_right = 100;
            if (Out_right < -100) Out_right = -100;
            
            // 当目标速度为0时的特殊处理 - 消除抖动
            if (Target == 0)
            {
                // 添加输出死区 - 小输出时直接置零
                if (fabs(Out) < 8)  // 增大死区范围
                {
                    Out = 0;
                }
                if (fabs(Out_right) < 8)
                {
                    Out_right = 0;
                }
                
                // 进一步：如果速度很小且输出很小，完全停止
                if (fabs(Actual) < 2 && fabs(Out) < 10)
                {
                    Out = 0;
                }
                if (fabs(Actual_right) < 2 && fabs(Out_right) < 10)
                {
                    Out_right = 0;
                }
            }
            else
            {
                // 非零目标时的死区补偿
                if (fabs(Out) < 12 && fabs(Error0) > 5)
                {
                    Out = (Out > 0) ? 12 : -12;
                }
                if (fabs(Out_right) < 12 && fabs(Error0_r) > 5)
                {
                    Out_right = (Out_right > 0) ? 12 : -12;
                }
            }
            
            // 最终输出前再次检查，确保零速时无抖动
            if (Target == 0 && fabs(Actual) < 3 && fabs(Actual_right) < 3)
            {
                // 如果实际速度接近零且目标为零，强制输出为零
                if (fabs(Out) < 15) Out = 0;
                if (fabs(Out_right) < 15) Out_right = 0;
            }
            
            Motor_SetPWM((int8_t)Out);
            Motor_SetPWM_right((int8_t)Out_right);
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

/* 速度控制模式 */
void Speed_control(void)
{
    Target = ExtractSpeed();
    
    /* OLED显示 */
    OLED_Printf(0, 16, OLED_8X16, "L:%+04.0f R:%+04.0f", Actual, Actual_right);
    OLED_Printf(0, 32, OLED_8X16, "Tar:%+04.0f", Target);
    OLED_Printf(0, 48, OLED_8X16, "Out:L%+03.0f R%+03.0f", Out, Out_right);
    
    // 显示状态信息
    if (Target < 0)
    {
        OLED_Printf(64, 16, OLED_8X16, "Direction:REV");
    }
    else if (Target > 0)
    {
        OLED_Printf(64, 16, OLED_8X16, "Direction:FWD");
    }
    else
    {
        OLED_Printf(64, 16, OLED_8X16, "Direction:STOP");
        // 显示防抖动状态
        OLED_Printf(64, 32, OLED_8X16, "Anti-Shake:ON ");
    }
    
    OLED_Printf(64, 48, OLED_8X16, "PosL:%+04d", (int)Left_Pos);
    
    /* 串口输出 */
    static uint32_t Tick = 0;
    if (Tick++ % 5 == 0)  // 进一步降低输出频率
    {
        Serial_Printf("%.0f,%.0f,%.0f\n", 
                       Target, Actual, Actual_right);
    }
}

/* 电机驱动模式 */
void Motor_drive(void)
{
    int16_t left_delta = Encoder_Get_Left();
    int16_t right_delta = Encoder_Get_Right();
    Encoder_Clear_Both();
    
    Actual_left += left_delta;
    Actual_right += right_delta;
    
    Target_right = Actual_left;
    
    // 位置式PID
    Error1_right = Error0_right;
    Error0_right = Target_right - Actual_right;
    
    // 积分分离
    if (fabs(Error0_right) < 80)
    {
        ErrorInt_right += Error0_right;
        ErrorInt_right = (ErrorInt_right > FOLLOW_INTEGRAL_LIMIT) ? FOLLOW_INTEGRAL_LIMIT :
                        ((ErrorInt_right < -FOLLOW_INTEGRAL_LIMIT) ? -FOLLOW_INTEGRAL_LIMIT : ErrorInt_right);
    }
    else
    {
        ErrorInt_right *= 0.9f;
    }
    
    // 位置PID计算
    Out_right = KP_FOLLOW * Error0_right + 
                KI_FOLLOW * ErrorInt_right + 
                KD_FOLLOW * (Error0_right - Error1_right);
    
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

/* 串口指令解析 */
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
                
                Serial_Printf("Set Speed: %d\n", last);
            }
        }
    }
    return last;
}