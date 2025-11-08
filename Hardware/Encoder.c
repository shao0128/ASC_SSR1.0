#include "Encoder.h"

// 静态变量存储上一次计数值，用于计算增量
static int16_t lastLeftCnt = 0;
static int16_t lastRightCnt = 0;

/**
  * 函    数：编码器初始化（TIM3左电机，TIM4右电机）
  * 参    数：无
  * 返 回 值：无
  * 配置说明：TI12模式（A/B相正交解码），上拉输入，滤波系数0xF
  */
void Encoder_Init(void)
{
    /* 开启时钟 */
    // 左编码器（TIM3：PA6=CH1，PA7=CH2）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    // 右编码器（TIM4：PB6=CH1，PB7=CH2）
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);

    /* GPIO初始化：上拉输入模式 */
    GPIO_InitTypeDef GPIO_InitStructure;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;

    // 左编码器GPIO（PA6、PA7）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
    // 右编码器GPIO（PB6、PB7）
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
    GPIO_Init(GPIOB, &GPIO_InitStructure);

    /* 时基单元初始化（16位向上计数，无预分频） */
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInitStructure.TIM_Period = 65535;  // 16位最大计数
    TIM_TimeBaseInitStructure.TIM_Prescaler = 0;    // 不分频
    TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;

    // 左编码器TIM3初始化
    TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);
    // 右编码器TIM4初始化
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    /* 输入捕获初始化（滤波配置） */
    TIM_ICInitTypeDef TIM_ICInitStructure;
    TIM_ICStructInit(&TIM_ICInitStructure);
    TIM_ICInitStructure.TIM_ICFilter = 0xF;  // 强滤波，抑制抖动

    // 左编码器TIM3通道1配置
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);
    // 左编码器TIM3通道2配置
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM3, &TIM_ICInitStructure);

    // 右编码器TIM4通道1配置
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);
    // 右编码器TIM4通道2配置
    TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;
    TIM_ICInit(TIM4, &TIM_ICInitStructure);

    /* 编码器接口配置（TI12模式，双通道不反相） */
    TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);
    TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Rising);

    /* 初始化计数值存储变量 */
    lastLeftCnt = TIM_GetCounter(TIM3);
    lastRightCnt = TIM_GetCounter(TIM4);

    /* 使能定时器 */
    TIM_Cmd(TIM3, ENABLE);
    TIM_Cmd(TIM4, ENABLE);
}

/**
  * 函    数：获取左编码器增量值（原子操作）
  * 参    数：无
  * 返 回 值：本次采样周期内的编码器计数增量（正为正转，负为反转）
  */
int16_t Encoder_GetLeftDelta(void)
{
    int16_t currentCnt = 0;
    int16_t delta = 0;

    __disable_irq();  // 关中断，避免计数读取时被打断

    currentCnt = TIM_GetCounter(TIM3);
    delta = currentCnt - lastLeftCnt;
    lastLeftCnt = currentCnt;

    // 处理16位计数器溢出
    if (delta > 32768)
        delta -= 65536;
    else if (delta < -32768)
        delta += 65536;

    __enable_irq();  // 开中断

    return delta;
}

/**
  * 函    数：获取右编码器增量值（原子操作）
  * 参    数：无
  * 返 回 值：本次采样周期内的编码器计数增量（正为正转，负为反转）
  */
int16_t Encoder_GetRightDelta(void)
{
    int16_t currentCnt = 0;
    int16_t delta = 0;

    __disable_irq();  // 关中断，避免计数读取时被打断

    currentCnt = TIM_GetCounter(TIM4);
    delta = currentCnt - lastRightCnt;
    lastRightCnt = currentCnt;

    // 处理16位计数器溢出
    if (delta > 32768)
        delta -= 65536;
    else if (delta < -32768)
        delta += 65536;

    __enable_irq();  // 开中断

    return delta;
}