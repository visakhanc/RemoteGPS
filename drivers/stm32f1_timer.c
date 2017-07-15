/**
 * @file stm32f1_timer.c
 * @brief Simple driver for STM32F1xx Timer
 * @author Visakhan
 * @date June 25, 2017
 */


#include "stm32f1_timer.h"

/**
 * @brief Start a timer with specified prescaler and reload value
 * @note Clock for the time should be enabled in RCC
 * @note The timer is configured to generate interrupt at reload value.
 * But additionally, timer interrupt need to be enabled at NVIC.
 * @param timer Timer base address
 * @param prescaler Prescaler value used to divide the timer input clock.
 * Prescaler register is written with (prescaler - 1)
 * @param reload Reload value to be configured
 */
void Timer_Init(TIM_TypeDef *timer, uint32_t prescaler, uint32_t reload)
{
	/* Set prescaler value */
	timer->PSC = prescaler - 1;
	/* Set reload value */
	timer->ARR = reload;
	/* Enable timer interrupt on update event(overflow) */
	timer->DIER = TIM_DIER_UIE;
	/* Enable timer */
	timer->CR1 = TIM_CR1_CEN;
}






