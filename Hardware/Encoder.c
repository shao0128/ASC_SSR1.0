#include "stm32f10x.h"                  
#include "Encoder.h"

/**
  * 函    数：编码器初始化
  * 参    数：无
  * 返 回 值：无
  */
void Encoder_Init(void)
{
	/*开启时钟*/
	//left
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);			//开启TIM3的时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);			//开启GPIOA的时钟
	//right
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	
	/*GPIO初始化*/
	//left
	GPIO_InitTypeDef GPIO_InitStructure;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);							//将PA6和PA7引脚初始化为上拉输入
	
	//right
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7;
	GPIO_Init(GPIOB, &GPIO_InitStructure);							//将PB6和PB7引脚初始化为上拉输入
	
	/*时基单元初始化*/
	//left
	TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;				//定义结构体变量
	TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;     //时钟分频，选择不分频，此参数用于配置滤波器时钟，不影响时基单元功能
	TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up; //计数器模式，选择向上计数
	TIM_TimeBaseInitStructure.TIM_Period = 65536 - 1;               //计数周期，即ARR的值
	TIM_TimeBaseInitStructure.TIM_Prescaler = 1 - 1;                //预分频器，即PSC的值
	TIM_TimeBaseInitStructure.TIM_RepetitionCounter = 0;            //重复计数器，高级定时器才会用到
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM3的时基单元
	
	//right
	TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);             //将结构体变量交给TIM_TimeBaseInit，配置TIM4的时基单元
	
	/*输入捕获初始化*/
	TIM_ICInitTypeDef TIM_ICInitStructure;							//定义结构体变量
	TIM_ICStructInit(&TIM_ICInitStructure);                         //结构体初始化，若结构体没有完整赋值
	                                                                //则最好执行此函数，给结构体所有成员都赋一个默认值
	//left                                                                //避免结构体初值不确定的问题
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //选择配置定时器通道1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                         //输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM3, &TIM_ICInitStructure);                         //将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;                //选择配置定时器通道2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                         //输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM3, &TIM_ICInitStructure);                         //将结构体变量交给TIM_ICInit，配置TIM3的输入捕获通道
	
	//right
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1;                //选择配置定时器通道1
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                         //输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM4, &TIM_ICInitStructure);                         //将结构体变量交给TIM_ICInit，配置TIM4的输入捕获通道
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2;                //选择配置定时器通道2
	TIM_ICInitStructure.TIM_ICFilter = 0xF;                         //输入滤波器参数，可以过滤信号抖动
	TIM_ICInit(TIM4, &TIM_ICInitStructure);                         //将结构体变量交给TIM_ICInit，配置TIM4的输入捕获通道
	
	/*编码器接口配置*/
	//left
	TIM_EncoderInterfaceConfig(TIM3, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
																	//配置编码器模式以及两个输入通道是否反相
	                                                                //注意此时参数的Rising和Falling已经不代表上升沿和下降沿了，而是代表是否反相
	                                                                //此函数必须在输入捕获初始化之后进行，否则输入捕获的配置会覆盖此函数的部分配置
	//right
	TIM_EncoderInterfaceConfig(TIM4, TIM_EncoderMode_TI12, TIM_ICPolarity_Rising, TIM_ICPolarity_Falling);
	
	/*TIM使能*/
	TIM_Cmd(TIM3, ENABLE);//left			//使能TIM3，定时器开始运行
	TIM_Cmd(TIM4, ENABLE);//right
}

// 在原有Encoder.c文件末尾添加以下函数：

/**
  * 函    数：安全读取左编码器值（原子操作）
  * 参    数：无
  * 返 回 值：左编码器的计数值
  */
int16_t Encoder_Get_Left(void)
{
    int16_t count;
    __disable_irq();                    // 关中断，保护关键操作
    count = TIM_GetCounter(TIM3);       // 读取编码器值
    __enable_irq();                     // 开中断
    return count;
}

/**
  * 函    数：安全读取右编码器值（原子操作）
  * 参    数：无
  * 返 回 值：右编码器的计数值
  */
int16_t Encoder_Get_Right(void)
{
    int16_t count;
    __disable_irq();                    // 关中断，保护关键操作
    count = TIM_GetCounter(TIM4);       // 读取编码器值
    __enable_irq();                     // 开中断
    return count;
}

/**
  * 函    数：同时清零两个编码器（原子操作）
  * 参    数：无
  * 返 回 值：无
  */
void Encoder_Clear_Both(void)
{
    __disable_irq();                    // 关中断，保护关键操作
    TIM_SetCounter(TIM3, 0);            // 清零左编码器
    TIM_SetCounter(TIM4, 0);            // 清零右编码器
    __enable_irq();                     // 开中断
}