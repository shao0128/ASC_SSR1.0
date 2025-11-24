#include "stm32f10x.h"                  // Device header




void Sensor_Init(void)
{
	// 使能 GPIOA 和 GPIOB 时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
    
  // 配置 PA4, PA5
  GPIO_InitTypeDef GPIO_InitStructure;  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4 | GPIO_Pin_5;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
    
  // 配置 PB0, PB1
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; // 上拉输入
  GPIO_Init(GPIOB, &GPIO_InitStructure);
}



uint8_t Route_Judge(void)
{
	static uint8_t Route_Flag;
	static uint8_t Flag_Time;
	
	Flag_Time++;
	
	if (Flag_Time>=2) //每隔20ms判断一次状态
	{
		Flag_Time=0;
		
		//直角左拐
		if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 0&&GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 1)
		{
			Route_Flag=4;                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                          
		}
		//直角右拐
		else if (GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0) == 1&&GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1) == 0)
		{
			Route_Flag=5;
		}
		//直行
		else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0&&GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)
		{
			Route_Flag=1;
		}
		//轻微左拐
		else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4) == 0)
		{
			Route_Flag=2;                                                                                                                                                                                                
		}
		//轻微右拐
		else if (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5) == 0)
		{
			Route_Flag=3;
		}                                                                                                                                                                                                                                                                                                                                                                                                       
	}
	
	return Route_Flag;
}