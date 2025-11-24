#include "stm32f10x.h"                  // Device header
#include "Encoder.h"
#include "PWM.h"
#include "Motor.h"
#include "Key.h"
#include "Serial.h"
#include "string.h"
#include <stdio.h>
#include <stdlib.h>
#include "Timer.h"
#include "OLED.h"
#include "PID.h"
#include "Sensor.h"
#include "Image.h"

#define Stringht     1
#define LightLeft    2
#define LightRight   3
#define StrLeft      4
#define StrRight     5

uint8_t Key_Num;   //小车启动按键

uint8_t Route_Flag;  //小车路线判断

//电机旋转速度的实际值
int16_t Speed1;
int16_t Speed2;




//电机目标速度
int16_t Speed_Target;


//提取接收到的数据包中的数字（速度）
int16_t ExtractSpeed(void);

int main(void)
{
	
	Moter_Init();
	Key_Init();
	Time_Init();
  Encoder_Init();
	Serial_Init();
	OLED_Init();
	Sensor_Init();
	
	while (1)
	{
		Image_Control();    //实现菜单调速功能
	}
		
}


void TIM2_IRQHandler(void)
{
	if (TIM_GetITStatus(TIM2, TIM_IT_Update) == SET)
	{
		static int16_t cnt;
		cnt++;
		
		if (cnt>=10)
		{
		
			cnt=0;
			//每10ms读取一次电机旋转速度实际值
		
			Speed1=Encoder_GetSpeed1();
		
			Speed2=Encoder_GetSpeed2();
		
		
			Key_Num=Key_GetNum();   //监视按键状态
		
		
			Route_Flag=Route_Judge();
			
		
			Speed_Target=Key_Speed();
			
		
		if (Key_Num==0) //按下按键小车启动
    {
		if (Route_Flag==Stringht)
		{
			PID_Straight();
		}
		else if (Route_Flag==LightLeft)                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                 
		{
			PID_LeftSlight();
		}
		else if (Route_Flag==LightRight)
		{
			PID_RightSlight();
		}
		else if (Route_Flag==StrLeft)
		{
			PID_LeftStraight();
		}
		else if (Route_Flag==StrRight)
		{
			PID_RightStraight();
		}
	  }
		
		if (Key_Num==1)     //按下按键小车静止
		{
			PID_Still();
		}
	
		}
		TIM_ClearITPendingBit(TIM2, TIM_IT_Update);
	}
}