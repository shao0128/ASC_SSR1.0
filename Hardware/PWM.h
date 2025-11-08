#ifndef __PWM_H
#define __PWM_H

#include "stm32f10x.h"

#define PWM_MAX 100  // PWM最大CCR值（对应ARR=99，占空比0~100%）

void PWM_Init(void);
void PWM_SetCompare2(uint16_t Compare);  // 右电机：TIM2_CH2（PA1）
void PWM_SetCompare3(uint16_t Compare);  // 左电机：TIM2_CH3（PA2）

#endif
