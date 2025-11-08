#include "PWM.h"

/**
  * 函    数：PWM初始化（TIM2生成双路PWM，控制左右电机）
  * 参    数：无
  * 返 回 值：无
  * 配置说明：TIM2_CH2(PA1)→右电机，TIM2_CH3(PA2)→左电机，频率10kHz，占空比0~100%
  */
void PWM_Init(void)
{
    /* 开启时钟 */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM2, ENABLE);  // 开启TIM2时钟
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); // 开启GPIOA时钟

    /* GPIO初始化：PA1(CH2)、PA2(CH3)配置为复用推挽输出 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1 | GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* 配置TIM2内部时钟 */
    TIM_InternalClockConfig(TIM2);

    /* 时基单元初始化 */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = PWM_MAX - 1;  // ARR=99，周期100个计数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 7200 - 1;  // PSC=7199，分频后时钟10kHz（72MHz/7200=10kHz）
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;
    TIM_TimeBaseInit(TIM2, &TIM_TimeBaseInitStructure);

    /* 输出比较初始化（通用配置） */
    TIM_OCInitTypeDef TIM_OCInitStructure;
    TIM_OCStructInit(&TIM_OCInitStructure);
    TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
    TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
    TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
    TIM_OCInitStructure.TIM_Pulse = 0;  // 初始占空比0%

    /* 配置右电机PWM通道（TIM2_CH2） */
    TIM_OC2Init(TIM2, &TIM_OCInitStructure);
    /* 配置左电机PWM通道（TIM2_CH3） */
    TIM_OC3Init(TIM2, &TIM_OCInitStructure);

    /* 使能TIM2 */
    TIM_Cmd(TIM2, ENABLE);
}

/**
  * 函    数：设置右电机PWM占空比（TIM2_CH2）
  * 参    数：Compare CCR值，范围0~PWM_MAX（对应0~100%占空比）
  * 返 回 值：无
  */
void PWM_SetCompare2(uint16_t Compare)
{
    // 限幅保护，避免超出PWM范围
    if (Compare > PWM_MAX) Compare = PWM_MAX;
    TIM_SetCompare2(TIM2, Compare);
}

/**
  * 函    数：设置左电机PWM占空比（TIM2_CH3）
  * 参    数：Compare CCR值，范围0~PWM_MAX（对应0~100%占空比）
  * 返 回 值：无
  */
void PWM_SetCompare3(uint16_t Compare)
{
    // 限幅保护，避免超出PWM范围
    if (Compare > PWM_MAX) Compare = PWM_MAX;
    TIM_SetCompare3(TIM2, Compare);
}
