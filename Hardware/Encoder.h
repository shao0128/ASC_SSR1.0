#ifndef __ENCODER_H
#define __ENCODER_H

#include "stm32f10x.h"

void Encoder_Init(void);
int16_t Encoder_GetLeftDelta(void);  // 获取左编码器增量值
int16_t Encoder_GetRightDelta(void); // 获取右编码器增量值

#endif

