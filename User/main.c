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

int16_t ExtractSpeed(void);
void Speed_control(void);
void Motor_drive(void);

/*电机测试*/
uint8_t KeyNum;
int16_t PWM;

uint8_t mode = 1;                       // 0: Motor drive, 1: Speed control


/*定义变量*/
float Target = 0, Actual, Out;           // 目标值，实际值，输出值
float Actual_left = 0;
float Target_right = 0, Actual_right = 0, Out_right = 0;  // 右电机变量
float Kp = 0.5, Ki = 0.05, Kd = 0.2;    // 速度PID参数
float Kp_pos = 0.3, Ki_pos = 0.02, Kd_pos = 0.1;  // 位置PID参数
float Error0, Error1, Error2;           // 速度误差
float Error0_right = 0, Error1_right = 0, ErrorInt_right = 0;  // 位置误差

// 添加右电机速度PID变量
float Error0_r = 0, Error1_r = 0, Error2_r = 0;  // 右电机速度PID误差

int main(void)
{
    /*模块初始化*/
    OLED_Init();
    Key_Init();
    Motor_Init();
    Encoder_Init();
    Serial_Init();
    Timer_Init();       // 10ms中断
    
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
                
                // 停止电机
                Motor_SetPWM(0);
                Motor_SetPWM_right(0);
            }
            else  // 切换到Speed control模式
            {
                // 重置速度PID变量
                Target = 0;
                Actual = 0;
                Out = 0;
                Error0 = 0;
                Error1 = 0;
                Error2 = 0;
				// 重置右电机速度PID变量
				Actual_right = 0;
				Out_right = 0;
				Error0_r = 0;
				Error1_r = 0;
				Error2_r = 0;
            }
            
            Delay_ms(200);  // 防抖延时
        }
        
        // 根据模式执行不同功能
        if (mode == 0)
        {
            OLED_Printf(0, 0, OLED_8X16, "Motor Drive     ");
            Motor_drive();
        }
        else
        {
            OLED_Printf(0, 0, OLED_8X16, "Speed Control   ");
            Speed_control();
        }
        
        OLED_Update();
        Delay_ms(10);
    }
}






void TIM1_UP_IRQHandler(void)
{
    static float Filtered_Actual = 0;           // 左电机滤波速度
    static float Filtered_Actual_right = 0;     // 右电机滤波速度
    
    if (TIM_GetITStatus(TIM1, TIM_IT_Update) == SET)
    {
        if (mode == 1)  // Speed control模式
        {            
            /* 关键修改：同时读取两个编码器，然后同时清零 */
            
            // 1. 先同时读取两个编码器的原始值
            int16_t raw_left = TIM_GetCounter(TIM3);     // 读取左电机TIM3
            int16_t raw_right = TIM_GetCounter(TIM4);    // 读取右电机TIM4
            
            // 2. 然后同时清零两个计数器
            TIM_SetCounter(TIM3, 0);
            TIM_SetCounter(TIM4, 0);
            
            /* 左电机速度处理 */
            // 3. 对左电机原始速度进行滤波
            Filtered_Actual = 0.6 * Filtered_Actual + 0.4 * raw_left;
            Actual = Filtered_Actual;
            
            // 4. 左电机PID计算
            Error2 = Error1;
            Error1 = Error0;
            Error0 = Target - Actual;
            
            float deltaOut = Kp * (Error0 - Error1) + Ki * Error0 + Kd * (Error0 - 2*Error1 + Error2);
            Out += deltaOut;
            
            // 输出限幅
            if (Out > 80) Out = 80;
            if (Out < -80) Out = -80;
            
            Motor_SetPWM(Out);  // 控制左电机
            
            /* 右电机速度处理 */
            // 5. 对右电机原始速度进行滤波
            Filtered_Actual_right = 0.6 * Filtered_Actual_right + 0.4 * raw_right;
            Actual_right = Filtered_Actual_right;
            
            // 6. 右电机PID计算
            Error2_r = Error1_r;
            Error1_r = Error0_r;
            Error0_r = Target - Actual_right;
            
            float deltaOut_r = Kp * (Error0_r - Error1_r) + Ki * Error0_r + Kd * (Error0_r - 2*Error1_r + Error2_r);
            Out_right += deltaOut_r;
            
            if (Out_right > 80) Out_right = 80;
            if (Out_right < -80) Out_right = -80;
            
            Motor_SetPWM_right(Out_right);  // 控制右电机
        }
        
        TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
    }
}


void Speed_control(void)
{
    Target = ExtractSpeed();
    
    /* 修改后的OLED显示 - 同时显示左右电机信息 */
    OLED_Printf(0, 16, OLED_8X16, "L:%+04.0f R:%+04.0f", Actual, Actual_right);
    OLED_Printf(0, 32, OLED_8X16, "Tar:%+04.0f", Target);
    OLED_Printf(0, 48, OLED_8X16, "Out:L%+03.0f R%+03.0f", Out, Out_right);
    
    // 可选：在第二行显示PID参数
    OLED_Printf(64, 16, OLED_8X16, "Kp:%4.2f", Kp);
    OLED_Printf(64, 32, OLED_8X16, "Ki:%4.2f", Ki);
    OLED_Printf(64, 48, OLED_8X16, "Kd:%4.2f", Kd);
    
    // 串口输出详细数据用于分析
    Serial_Printf("Target=%.0f, Left=%.0f(Out=%.0f), Right=%.0f(Out=%.0f)\n", 
                  Target, Actual, Out, Actual_right, Out_right);
}


void Motor_drive(void)
{
    // 读取位置增量
    Actual_left += Encoder_Get();
    Actual_right += Encoder_Get_right();
    
    Target_right = Actual_left;
    
    // 位置PID计算
    Error1_right = Error0_right;
    Error0_right = Target_right - Actual_right;
    
    // 积分分离
    if (fabs(Error0_right) < 100)
    {
        ErrorInt_right += Error0_right;
        // 积分限幅
        if (ErrorInt_right > 500) ErrorInt_right = 500;
        if (ErrorInt_right < -500) ErrorInt_right = -500;
    }
    else
    {
        ErrorInt_right = 0;
    }
    
    // 位置PID
    Out_right = Kp_pos * Error0_right + Ki_pos * ErrorInt_right + Kd_pos * (Error0_right - Error1_right);
    
    // 输出限幅
    if (Out_right > 80) Out_right = 80;
    if (Out_right < -80) Out_right = -80;
    
    // 死区补偿
    if (fabs(Out_right) < 15 && fabs(Error0_right) > 20) 
    {
        Out_right = (Out_right > 0) ? 20 : -20;
    }
    
    // 控制右电机
    Motor_SetPWM_right(Out_right);
    
    // 显示信息
    OLED_Printf(0, 16, OLED_8X16, "L:%+06.0f", Actual_left);
    OLED_Printf(0, 32, OLED_8X16, "R:%+06.0f", Actual_right);
    OLED_Printf(0, 48, OLED_8X16, "Out:%+03.0f Err:%+04.0f", Out_right, Error0_right);
    
    Serial_Printf("%f,%f,%f,%f\r\n", Actual_left, Target_right, Actual_right, Out_right);
}

int16_t ExtractSpeed(void)
{
    static int16_t last = 0;
    if (Serial_RxFlag == 1)
    {
        Serial_RxFlag = 0;
        
        Serial_Printf("Received: %s\r\n", Serial_RxPacket);
        int8_t sign = 1;
        uint8_t start_index = 0;
        
        if (Serial_RxPacket[0] == '+')
        {
            sign = 1;
            start_index = 1;
            Serial_Printf("Direction: Positive\r\n");
        }
        else if (Serial_RxPacket[0] == '-')
        {
            sign = -1;
            start_index = 1;
            Serial_Printf("Direction: Negative\r\n");
        }
        
        // 解析数字部分
        uint16_t speed = 0;
        for (uint8_t i = start_index; Serial_RxPacket[i] != '\0'; i++)
        {
            if (Serial_RxPacket[i] >= '0' && Serial_RxPacket[i] <= '9')
            {
                speed = speed * 10 + (Serial_RxPacket[i] - '0');
            }
        }
        
        last = sign * speed;
        Serial_Printf("Final Speed: %d\r\n", last);
    }
    return last;
}