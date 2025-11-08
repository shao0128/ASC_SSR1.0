#include "Motor.h"
#include "PWM.h"

/**
  * 函    数：电机初始化（控制引脚+PWM初始化）
  * 参    数：无
  * 返 回 值：无
  * 配置说明：左电机(PB12=IN1, PB13=IN2)，右电机(PB14=IN1, PB15=IN2)，初始状态停止
  */
void Motor_Init(void)
{
    /* 开启GPIOB时钟 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* GPIO初始化：电机方向控制引脚（推挽输出） */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 初始化电机为停止状态（IN1=IN2=低电平，适配L298N芯片） */
    GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15);

    /* 初始化底层PWM模块 */
    PWM_Init();
}

/**
  * 函    数：设置左电机PWM（含方向控制）
  * 参    数：pwm - 范围-MOTOR_PWM_MAX~MOTOR_PWM_MAX（正=正转，负=反转，0=停止）
  * 返 回 值：无
  */
void Motor_SetLeftPWM(int8_t pwm)
{
    uint16_t pwmVal = (pwm >= 0) ? pwm : -pwm;  // 转为无符号PWM值

    if (pwm > 0)
    {
        // 正转：PB12=低，PB13=高
        GPIO_ResetBits(GPIOB, GPIO_Pin_12);
        GPIO_SetBits(GPIOB, GPIO_Pin_13);
        PWM_SetCompare3(pwmVal);
    }
    else if (pwm < 0)
    {
        // 反转：PB12=高，PB13=低
        GPIO_SetBits(GPIOB, GPIO_Pin_12);
        GPIO_ResetBits(GPIOB, GPIO_Pin_13);
        PWM_SetCompare3(pwmVal);
    }
    else
    {
        // 停止：PB12=低，PB13=低，PWM=0
        GPIO_ResetBits(GPIOB, GPIO_Pin_12 | GPIO_Pin_13);
        PWM_SetCompare3(0);
    }
}

/**
  * 函    数：设置右电机PWM（含方向控制）
  * 参    数：pwm - 范围-MOTOR_PWM_MAX~MOTOR_PWM_MAX（正=正转，负=反转，0=停止）
  * 返 回 值：无
  */
void Motor_SetRightPWM(int8_t pwm)
{
    uint16_t pwmVal = (pwm >= 0) ? pwm : -pwm;  // 转为无符号PWM值

    if (pwm > 0)
    {
        // 正转：PB14=低，PB15=高
        GPIO_ResetBits(GPIOB, GPIO_Pin_14);
        GPIO_SetBits(GPIOB, GPIO_Pin_15);
        PWM_SetCompare2(pwmVal);
    }
    else if (pwm < 0)
    {
        // 反转：PB14=高，PB15=低
        GPIO_SetBits(GPIOB, GPIO_Pin_14);
        GPIO_ResetBits(GPIOB, GPIO_Pin_15);
        PWM_SetCompare2(pwmVal);
    }
    else
    {
        // 停止：PB14=低，PB15=低，PWM=0
        GPIO_ResetBits(GPIOB, GPIO_Pin_14 | GPIO_Pin_15);
        PWM_SetCompare2(0);
    }
}

/**
  * 函    数：停止所有电机
  * 参    数：无
  * 返 回 值：无
  */
void Motor_StopAll(void)
{
    Motor_SetLeftPWM(0);
    Motor_SetRightPWM(0);
}