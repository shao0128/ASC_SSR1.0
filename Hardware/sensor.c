#include "stm32f10x.h"                  // Device header
#include "sensor.h"

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
    static uint8_t Route_Flag = 1; // 默认直行
    static uint8_t Flag_Time = 0;
    
    Flag_Time++;
    
    if (Flag_Time >= 2) // 每隔20ms判断一次状态
    {
        Flag_Time = 0;
        
        uint8_t PA4 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_4);
        uint8_t PA5 = GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_5);
        uint8_t PB0 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_0);
        uint8_t PB1 = GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_1);
        
        // 直角左拐 - 最外侧传感器检测到黑线
        if (PB0 == 0 && PB1 == 1)
        {
            Route_Flag = 4;
        }
        // 直角右拐 - 最外侧传感器检测到黑线
        else if (PB0 == 1 && PB1 == 0)
        {
            Route_Flag = 5;
        }
        // 直行 - 中间两个传感器都检测到黑线
        else if (PA4 == 0 && PA5 == 0)
        {
            Route_Flag = 1;
        }
        // 轻微左拐 - 左侧中间传感器检测到黑线
        else if (PA4 == 0 && PA5 == 1)
        {
            Route_Flag = 2;
        }
        // 轻微右拐 - 右侧中间传感器检测到黑线
        else if (PA4 == 1 && PA5 == 0)
        {
            Route_Flag = 3;
        }
        // 丢失路径 - 所有传感器都没检测到黑线，保持上次状态
        else if (PA4 == 1 && PA5 == 1 && PB0 == 1 && PB1 == 1)
        {
            // 保持当前Route_Flag不变，继续上次的控制策略
        }
    }
    
    return Route_Flag;
}