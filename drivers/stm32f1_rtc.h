/**
 * @file	stm32f1_rtc.h
 * @brief	Driver to access RTC peripheral on STM32F1 device
 * @author	Visakhan
 * @date	May 25, 2017
 */

#ifndef STM32F1_RTC_H_
#define STM32F1_RTC_H_

#include <stdint.h>
#include <stdbool.h>

void RTC_Init(void);
void RTC_Init_LSI(void);
void RTC_Alarm_Config(uint32_t val);
void RTC_Set_Counter(uint32_t count);
uint32_t RTC_Get_Count(void);

#endif /* STM32F1_RTC_H_ */
