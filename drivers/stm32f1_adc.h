/*
 * stm32f1_adc.h
 *
 *  Created on: Apr 19, 2017
 *      Author: Visakhan
 */

#ifndef STM32F1_ADC_H_
#define STM32F1_ADC_H_


void Adc_Init(void);
void Adc_Enable(void);
void Adc_Disble(void);
uint16_t Adc_Sample_Polled(uint32_t channel);

#endif /* STM32F1_ADC_H_ */
