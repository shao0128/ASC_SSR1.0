#ifndef __MOTOR_H
#define __MOTOR_H

#include "stm32f10x.h"

#define MOTOR_PWM_MAX PWM_MAX  // 电机PWM最大范围（与PWM模块同步）

void Motor_Init(void);
void Motor_SetLeftPWM(int8_t pwm);   // 设置左电机PWM（-MOTOR_PWM_MAX~MOTOR_PWM_MAX）
void Motor_SetRightPWM(int8_t pwm);  // 设置右电机PWM（-MOTOR_PWM_MAX~MOTOR_PWM_MAX）
void Motor_StopAll(void);            // 停止所有电机

#endif
