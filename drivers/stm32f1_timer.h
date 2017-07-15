/**
 * @file stm32f1_timer.h
 * @brief Simple driver for STM32F1xx Timer
 * @author Visakhan
 * @date June 25, 2017
 */


#ifndef STM32F1_TIMER_H_
#define STM32F1_TIMER_H_

#include "stm32f1xx.h"


void Timer_Init(TIM_TypeDef *timer, uint32_t prescaler, uint32_t reload);

#endif /* STM32F1_TIMER_H_ */
