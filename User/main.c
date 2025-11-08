#include "stm32f10x.h"                  
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
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

/* PID参数优化（适配任务要求：响应快、超调小、抗干扰） */
#define KP_SPEED 0.8f    // 增强比例响应
#define KI_SPEED 0.05f    // 减小积分超调
#define KD_SPEED 0.15f   // 增强微分抗干扰
#define INTEGRAL_LIMIT 15// 积分限幅（防止饱和）
#define KP_POS 0.5f      // 位置PID比例系数
#define KI_POS 0.01f     // 位置PID积分系数
#define KD_POS 0.2f      // 位置PID微分系数

/* 控制变量定义 */
float Target = 0, Actual = 0, Out = 0;           // 左电机：目标值，实际值，输出值
float Actual_left = 0;
float Target_right = 0, Actual_right = 0, Out_right = 0;  // 右电机变量
float Error0 = 0, Error1 = 0, Error2 = 0;           // 左电机速度误差
float Error0_r = 0, Error1_r = 0, Error2_r = 0;     // 右电机速度误差
float Error0_right = 0, Error1_right = 0, ErrorInt_right = 0;  // 位置误差
static float Integral = 0, Integral_r = 0;          // 积分项存储（抗饱和）
static int32_t Left_Pos = 0, Right_Pos = 0;         // 累计位置（用于回正力逻辑）

/* 新增变量：速度控制相关 */
float LastTarget = 0;      // 保存上一次的目标速度
uint8_t NewSpeedCommand = 0; // 新速度指令标志

int main(void)
{
    /* 模块初始化 */
    OLED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Serial_Init();
    Timer_Init();       // 10ms中断
	
	    /* 关键：系统状态清零 */
    Encoder_Clear_Both();
    Motor_SetPWM(0);
    Motor_SetPWM_right(0);
    
    // 重置所有PID变量
    Target = 0; Actual = 0; Out = 0;
    Actual_left = 0; Target_right = 0; Actual_right = 0; Out_right = 0;
    Error0 = Error1 = Error2 = 0;
    Error0_r = Error1_r = Error2_r = 0;
    Error0_right = Error1_right = ErrorInt_right = 0;
    Integral = Integral_r = 0;
    Left_Pos = Right_Pos = 0;
    LastTarget = 0;
    NewSpeedCommand = 0;

    OLED_Printf(0, 0, OLED_8X16, "Init Complete!");
    OLED_Update();
    Delay_ms(1000);
    
    while (1)
    {
        KeyNum = Key_GetNum();
        
        // 按键切换模式
        if (KeyNum == 1)
        {
            mode = !mode;
            
            if (mode == 0)  // 切换到Motor drive模式
            {
                // 重置位置相关变量
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
                LastTarget = 0;
                NewSpeedCommand = 0;
                
                // 停止电机
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
                OLED_Clear();
                OLED_Printf(0, 0, OLED_8X16, "Mode: Motor Drive");
            }
            else  // 切换到Speed control模式
            {
                // 关键：强制重置目标速度为0
                Target = 0;
                Target_right = 0;  // 同时重置右电机目标速度
                LastTarget = 0;
                NewSpeedCommand = 0;  // 清除任何待处理的速度指令
                
                // 重置速度PID变量
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
                
                // 停止电机
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
                OLED_Clear();
                OLED_Printf(0, 0, OLED_8X16, "Mode: Speed Control");
                
                // 串口输出模式切换信息
                Serial_Printf("Switched to Speed Control Mode, Both Motors Target Speed Reset to 0\n");
            }
            OLED_Update();
            Delay_ms(200);  // 防抖延时
        }
        
        // 根据模式执行不同功能
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

/* 10ms定时器中断：PID计算核心 */
void TIM1_UP_IRQHandler(void)
{
    static float Filtered_Actual = 0;           // 左电机滤波速度
    static float Filtered_Actual_right = 0;     // 右电机滤波速度
    
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (mode == 1)  // Speed control模式（重点适配任务要求）
        {            
            /* 1. 原子操作读取+清零编码器（避免中断冲突） */
            int16_t raw_left = Encoder_Get_Left();     // 安全读取左电机TIM3
            int16_t raw_right = Encoder_Get_Right();   // 安全读取右电机TIM4
            Encoder_Clear_Both();
            
            /* 2. 左电机速度处理（含回正力逻辑+PID优化） */
			
            // 一阶滤波（平滑速度波动）
            Filtered_Actual = 0.6f * Filtered_Actual + 0.4f * raw_left;
            Actual = Filtered_Actual;
            
            // 回正力逻辑：目标速度=0时，基于位置偏差补偿
            float Pos_Compensate = 0;
            if (Target == 0)
            {
                Left_Pos += raw_left;  // 累计位置偏差
                Pos_Compensate = -KP_POS * Left_Pos;  // 位置负反馈（回正力）
                Pos_Compensate = Pos_Compensate > 30 ? 30 : (Pos_Compensate < -30 ? -30 : Pos_Compensate);
            }
            else
            {
                Left_Pos = 0;  // 非回正模式，清零位置累积
            }
            
            // 增量式PID计算（含积分限幅）
            Error2 = Error1;
            Error1 = Error0;
            Error0 = Target - Actual;
            Integral += Error0;
            // 积分限幅（抗饱和）
            Integral = Integral > INTEGRAL_LIMIT ? INTEGRAL_LIMIT : (Integral < -INTEGRAL_LIMIT ? -INTEGRAL_LIMIT : Integral);
            // PID输出（叠加位置补偿）
            float deltaOut = KP_SPEED * (Error0 - Error1) + KI_SPEED * Integral + KD_SPEED * (Error0 - 2*Error1 + Error2);
            Out += deltaOut + Pos_Compensate;
            
            // 输出限幅+死区补偿（避免低速抖动）
            if (Out > 80) Out = 80;
			if (Out < -80) Out = -80;
			// 更智能的死区补偿
			if (fabs(Out) < 12 && fabs(Error0) > 3) {
			Out = (Out > 0) ? 12 : -12;
			} else if (fabs(Out) < 8 && Target != 0) {
			Out = (Target > 0) ? 8 : -8;  // 根据目标方向补偿
			}
            
            Motor_SetPWM((int8_t)Out);  // 控制左电机
            
            /* 3. 右电机速度处理（与左电机同步，使用相同的目标速度） */
            Filtered_Actual_right = 0.6f * Filtered_Actual_right + 0.4f * raw_right;
            Actual_right = Filtered_Actual_right;
            
            // 右电机回正力逻辑
            float Pos_Compensate_r = 0;
            if (Target_right == 0)  // 使用相同的目标速度判断
            {
                Right_Pos += raw_right;  // 右电机位置偏差
                Pos_Compensate_r = -KP_POS * Right_Pos;  // 右电机回正力
                Pos_Compensate_r = Pos_Compensate_r > 30 ? 30 : (Pos_Compensate_r < -30 ? -30 : Pos_Compensate_r);
            }
            else
            {
                Right_Pos = 0;  // 非回正模式，清零位置累积
            }
            
            // 右电机PID计算（使用相同的目标速度Target）
            Error2_r = Error1_r;
            Error1_r = Error0_r;
            Error0_r = Target - Actual_right;  // 使用相同的目标速度
            Integral_r += Error0_r;
            Integral_r = Integral_r > INTEGRAL_LIMIT ? INTEGRAL_LIMIT : (Integral_r < -INTEGRAL_LIMIT ? -INTEGRAL_LIMIT : Integral_r);
            float deltaOut_r = KP_SPEED * (Error0_r - Error1_r) + KI_SPEED * Integral_r + KD_SPEED * (Error0_r - 2*Error1_r + Error2_r);
            Out_right += deltaOut_r + Pos_Compensate_r;
            
            // 输出限幅+死区补偿
            if (Out_right > 80) Out_right = 80;
            if (Out_right < -80) Out_right = -80;
            if (fabs(Out_right) < 15 && fabs(Error0_r) > 5)
            {
                Out_right = (Out_right > 0) ? 15 : -15;
            }
            
            Motor_SetPWM_right((int8_t)Out_right);  // 控制右电机
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

/* 速度控制模式：OLED显示+串口交互 */
void Speed_control(void)
{
    // 解析串口指令获取目标速度
    int16_t newSpeed = ExtractSpeed();
    
    // 如果有新的速度指令，更新两个电机的目标速度
    if (NewSpeedCommand)
    {
        Target = newSpeed;
        Target_right = newSpeed;  // 同时设置右电机目标速度
        LastTarget = Target;  // 保存当前目标值
        NewSpeedCommand = 0;  // 清除标志位
        Serial_Printf("New Target Speed Set for Both Motors: %d\n", (int)Target);
    }
    
    // 确保在没有新指令时保持当前目标速度
    
    /* OLED显示优化：清晰展示关键参数 */
    OLED_Printf(0, 16, OLED_8X16, "L:%+04.0f R:%+04.0f", Actual, Actual_right);
    OLED_Printf(0, 32, OLED_8X16, "Tar:%+04.0f", Target);
    OLED_Printf(0, 48, OLED_8X16, "Out:L%+03.0f R%+03.0f", Out, Out_right);
    
    // 显示当前目标状态
    OLED_Printf(64, 0, OLED_8X16, "Cmd:%d", NewSpeedCommand);
    
    // PID参数显示（方便调试）
    OLED_Printf(64, 16, OLED_8X16, "Kp:%.2f", KP_SPEED);
    OLED_Printf(64, 32, OLED_8X16, "Ki:%.2f", KI_SPEED);
    OLED_Printf(64, 48, OLED_8X16, "Kd:%.2f", KD_SPEED);
    
    /* 串口输出：适配上位机波形分析（带时间戳） */
    static uint32_t Tick = 0;
    Serial_Printf("%u,%.0f,%.0f,%.0f,%.0f,%.0f\n", 
              Tick++, Target, Actual, Out, Actual_right, Out_right);
	
}

/* 电机驱动模式：左电机位置→右电机目标（同步跟随） */
void Motor_drive(void)
{
    // 安全读取编码器位置
    Actual_left += Encoder_Get_Left();
    Actual_right += Encoder_Get_Right();
	
    Encoder_Clear_Both();
    
    Target_right = Actual_left;  // 左电机位置作为右电机目标
    
    // 位置式PID（积分分离+限幅）
    Error1_right = Error0_right;
    Error0_right = Target_right - Actual_right;
    
    // 积分分离（误差小时才积分）
    if (fabs(Error0_right) < 100)
    {
        ErrorInt_right += Error0_right;
        ErrorInt_right = ErrorInt_right > 500 ? 500 : (ErrorInt_right < -500 ? -500 : ErrorInt_right);
    }
    else
    {
        ErrorInt_right = 0;
    }
    
    // 位置PID计算
    Out_right = KP_POS * Error0_right + KI_POS * ErrorInt_right + KD_POS * (Error0_right - Error1_right);
    
    // 输出限幅+死区补偿
    if (Target != 0) {  // 仅在有目标速度时启用死区补偿
    if (fabs(Out) < 12 && fabs(Error0) > 3) {
        Out = (Out > 0) ? 12 : -12;
    } else if (fabs(Out) < 8) {
        Out = (Target > 0) ? 8 : -8;
    }
}
    
    Motor_SetPWM_right(Out_right);  // 控制右电机
    
    // OLED显示
    OLED_Printf(0, 16, OLED_8X16, "L_Pos:%+06.0f", Actual_left);
    OLED_Printf(0, 32, OLED_8X16, "R_Pos:%+06.0f", Actual_right);
    OLED_Printf(0, 48, OLED_8X16, "Out:%+03.0f Err:%+04.0f", Out_right, Error0_right);
	
    
    // 串口输出
    Serial_Printf("L_Pos=%.0f,R_Tar=%.0f,R_Pos=%.0f,R_Out=%.0f\n", 
                  Actual_left, Target_right, Actual_right, Out_right);
	
}

/* 串口指令解析：适配任务要求的@speed% 格式 */
int16_t ExtractSpeed(void)
{
    static int16_t lastParsedSpeed = 0;
    
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        Serial_Printf("Received: %s\n", Serial_RxPacket);
        
        // 识别指令前缀：@speed%（支持带空格/不带空格，如@speed%50 或 @speed% 50）
        char *cmd_prefix = strstr(Serial_RxPacket, "@speed%");
        if (cmd_prefix != NULL)
        {
            cmd_prefix += 7;  // 跳过"@speed%"（7个字符）
            // 跳过空格（如指令是@speed% 50，跳过空格到数字）
            while (*cmd_prefix == ' ' && *cmd_prefix != '\0') cmd_prefix++;
            
            // 解析正负号
            int8_t sign = 1;
            if (*cmd_prefix == '-')
            {
                sign = -1;
                cmd_prefix++;
            }
            else if (*cmd_prefix == '+')
            {
                sign = 1;
                cmd_prefix++;
            }
            
            // 解析数字部分
            uint16_t speed = 0;
            while (*cmd_prefix >= '0' && *cmd_prefix <= '9' && *cmd_prefix != '\0')
            {
                speed = speed * 10 + (*cmd_prefix - '0');
                cmd_prefix++;
            }
            
            // 限制速度范围（-100~100，避免电机过载）
            lastParsedSpeed = sign * speed;
            if (lastParsedSpeed > 100) lastParsedSpeed = 100;
            if (lastParsedSpeed < -100) lastParsedSpeed = -100;
            
            // 设置新速度指令标志
            NewSpeedCommand = 1;
            
            Serial_Printf("Parsed Speed: %d, Cmd Flag: %d\n", lastParsedSpeed, NewSpeedCommand);
        }
        else
        {
            // 无效指令提示
            Serial_Printf("Invalid Command! Use format: @speed% [value]\n");
            Serial_Printf("Example: @speed% 50 (forward), @speed% -40 (reverse)\n");
        }
    }
    return lastParsedSpeed;
}