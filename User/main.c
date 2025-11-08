#include "stm32f10x.h"                  
#include "Delay.h"
#include "OLED.h"
#include "LED.h"
#include "Timer.h"
#include "Key.h"
#include "Motor.h"  // 适配新电机驱动头文件
#include "Encoder.h"// 适配新编码器驱动头文件
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

/* PID参数 - 使用更简单的参数 */
#define KP_SPEED 0.3f    // 比例系数
#define KI_SPEED 0.05f   // 积分系数  
#define KD_SPEED 0.02f   // 微分系数
#define INTEGRAL_LIMIT 30// 积分限幅




/* 位置跟随PI参数（适配跟随模式） */
#define KP_POS 0.5f     // 位置比例系数
#define KI_POS 0.02f    // 位置积分系数
#define POS_INTEGRAL_LIMIT 100  // 位置积分限幅
static float Pos_Integral = 0;  // 位置积分项存储



/* 控制变量定义 */
float Target = 0, Actual = 0, Out = 0;           // 左电机：目标值，实际值，输出值
float Actual_left = 0;
float Actual_right = 0, Out_right = 0;           // 右电机变量
float Error0 = 0, Error1 = 0, Error2 = 0;           // 左电机速度误差
float Error0_r = 0, Error1_r = 0, Error2_r = 0;     // 右电机速度误差
static float Integral = 0, Integral_r = 0;          // 积分项存储

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
        
        // 按键切换模式
        if (KeyNum == 1)
        {
            mode = !mode;
            
            if (mode == 0)  // 切换到Motor drive模式
            {
                // 重置变量
                Actual_left = 0;
                Actual_right = 0;
                Integral = 0;
                Integral_r = 0;
                Pos_Integral = 0;
                // 停止电机（使用新函数）
                Motor_StopAll();
                OLED_Clear();
                OLED_Printf(0, 0, OLED_8X16, "Mode: Motor Drive");
            }
            else  // 切换到Speed control模式
            {
                // 重置变量
                Target = 0;
                Actual = 0;
                Out = 0;
                Error0 = 0;
                Error1 = 0;
                Error2 = 0;
                Actual_right = 0;
                Out_right = 0;
                Integral = 0;
                Integral_r = 0;
                
                // 停止电机（使用新函数）
                Motor_StopAll();
                OLED_Clear();
                OLED_Printf(0, 0, OLED_8X16, "Mode: Speed Control");
                OLED_Printf(0, 16, OLED_8X16, "Right Motor: FIXED");
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
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (mode == 1)  // Speed control模式
        {            
            /* 1. 读取编码器增量（新驱动无需清零） */
            int16_t raw_left = Encoder_GetLeftDelta();  // 新函数：左编码器增量
            int16_t raw_right = Encoder_GetRightDelta();// 新函数：右编码器增量
            
            /* 2. 左电机速度控制 */
            Actual = raw_left;  // 直接使用原始值，不过滤
            
            // 增量式PID计算
            Error2 = Error1;
            Error1 = Error0;
            Error0 = Target - Actual;
            
            // 积分项处理
            Integral += Error0;
            // 积分限幅
            if (Integral > INTEGRAL_LIMIT) Integral = INTEGRAL_LIMIT;
            if (Integral < -INTEGRAL_LIMIT) Integral = -INTEGRAL_LIMIT;
            
            // PID输出
            float deltaOut = KP_SPEED * (Error0 - Error1) + 
                           KI_SPEED * Integral + 
                           KD_SPEED * (Error0 - 2*Error1 + Error2);
            Out += deltaOut;
            
            // 输出限幅
            if (Out > 100) Out = 100;
            if (Out < -100) Out = -100;
            
            // 简单的死区处理
            if (fabs(Out) < 5 && fabs(Error0) < 5)
            {
                Out = 0;
            }
            else if (fabs(Out) < 10 && fabs(Error0) > 5)
            {
                Out = (Out > 0) ? 10 : -10;
            }
            
            Motor_SetLeftPWM((int8_t)Out);  // 新函数：控制左电机
            
            /* 3. 右电机保持静止（简化逻辑，直接设PWM为0） */
            Actual_right = raw_right;
            Motor_SetRightPWM(0);  // 新函数：右电机停止
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}

/* 速度控制模式：OLED显示+串口交互 */
void Speed_control(void)
{
    Target = ExtractSpeed();  // 解析串口指令获取目标速度（仅用于左电机）
    
    /* OLED显示 */
    OLED_Printf(0, 16, OLED_8X16, "L:%+04.0f R:%+04.0f", Actual, Actual_right);
    OLED_Printf(0, 32, OLED_8X16, "Tar:%+04.0f", Target);
    OLED_Printf(0, 48, OLED_8X16, "Out:L%+03.0f R:000", Out);  // 右电机固定为0
    
    // PID参数显示
    OLED_Printf(64, 16, OLED_8X16, "Kp:%.2f", KP_SPEED);
    OLED_Printf(64, 32, OLED_8X16, "Ki:%.2f", KI_SPEED);
    OLED_Printf(64, 48, OLED_8X16, "Kd:%.2f", KD_SPEED);
    
    /* 串口输出 */
    static uint32_t Tick = 0;
    Serial_Printf("%u,%.0f,%.0f,%.0f\n", 
                  Tick++, Target, Actual, Actual_right);
}

/* 电机驱动模式：左电机位置→右电机目标（同步跟随） */
/* 电机驱动模式：手转左电机，右电机位置同步跟随 */
void Motor_drive(void)
{
    // 读取编码器增量，累加得到左/右电机当前位置（手转左电机时，left_pos会变化）
    Actual_left += Encoder_GetLeftDelta();  // 左电机位置（手转产生的累积值）
    Actual_right += Encoder_GetRightDelta();// 右电机当前位置（反馈值）
    
    // 位置跟随闭环控制：以左电机位置为目标，右电机追踪
    float pos_error = Actual_left - Actual_right;  // 位置误差（目标-反馈）
    
    // PI控制计算（比例+积分，消除静态误差）
    Pos_Integral += pos_error;
    // 积分限幅，防止积分饱和
    if (Pos_Integral > POS_INTEGRAL_LIMIT) Pos_Integral = POS_INTEGRAL_LIMIT;
    if (Pos_Integral < -POS_INTEGRAL_LIMIT) Pos_Integral = -POS_INTEGRAL_LIMIT;
    
    // PI输出（控制右电机PWM）
    Out_right = KP_POS * pos_error + KI_POS * Pos_Integral;
    
    // 输出限幅（避免电机过载，范围±80）
    if (Out_right > 80) Out_right = 80;
    if (Out_right < -80) Out_right = -80;
    
    // 死区处理（避免低速抖动）
    if (fabs(Out_right) < 10 && fabs(pos_error) < 5)
    {
        Out_right = 0;
        Pos_Integral = 0;  // 误差过小时清零积分，避免累积
    }
    else if (fabs(Out_right) < 10 && fabs(pos_error) > 5)
    {
        Out_right = (Out_right > 0) ? 10 : -10;
    }
    
    Motor_SetRightPWM((int8_t)Out_right);  // 驱动右电机跟随
    
    // OLED显示优化：突出位置和误差
    OLED_Printf(0, 16, OLED_8X16, "L_Pos:%+06.0f", Actual_left);
    OLED_Printf(0, 32, OLED_8X16, "R_Pos:%+06.0f", Actual_right);
    OLED_Printf(0, 48, OLED_8X16, "Out:%+03.0f Err:%+04.0f", Out_right, pos_error);
    
    // 串口输出位置数据，便于调试
    Serial_Printf("L_Pos=%.0f,R_Pos=%.0f,Err=%.0f,R_Out=%.0f\n", 
                  Actual_left, Actual_right, pos_error, Out_right);
}
/* 串口指令解析：适配任务要求的@speed% 格式 */
int16_t ExtractSpeed(void)
{
    static int16_t last = 0;
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        Serial_Printf("Received: %s\n", Serial_RxPacket);
        
        // 识别指令前缀：@speed%
        char *cmd_prefix = strstr(Serial_RxPacket, "@speed%");
        if (cmd_prefix != NULL)
        {
            cmd_prefix += 7;  // 跳过"@speed%"
            // 跳过空格
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
            
            // 限制速度范围
            last = sign * speed;
            if (last > 100) last = 100;
            if (last < -100) last = -100;
            
            Serial_Printf("Set Left Target Speed: %d\n", last);
        }
        else
        {
            // 无效指令提示
            Serial_Printf("Invalid Command! Use: @speed% [value]\n");
        }
    }
    return last;
}