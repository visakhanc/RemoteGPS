/*
 * 	stm32f1_adc.c
 *
 *  Created on: Apr 18, 2017
 *  Author: Visakhan C
 */

#include "stm32f1xx_hal.h"
#include "stm32f1_adc.h"

void Adc_Init(void)
{
	/* Enable ADC and set SWSTART as trigger for ADC sampling */
	ADC1->CR2 = (ADC_CR2_ADON | ADC_CR2_EXTSEL );

	/* Calibration */
	ADC1->CR2 |= (ADC_CR2_CAL);
	while(ADC1->CR2 & ADC_CR2_CAL)
		;

	/* set sampling time = 41.5 cycles */
	ADC1->SMPR2 = 0x4;
}

uint16_t Adc_Sample_Polled(uint32_t channel)
{
	/* Clear any pending EOC flag */
	uint32_t adc_val = ADC1->DR;

	/* Set channel number in sequence register */
	ADC1->SQR3 = channel & ADC_SQR3_SQ1;

	/* Start conversion by SWSTART */
	ADC1->CR2 |= (ADC_CR2_SWSTART | ADC_CR2_EXTTRIG);

	while(!(ADC1->SR & ADC_SR_EOC))
		;

	return (uint16_t)(adc_val = ADC1->DR);
}


void Adc_Enable(void)
{
	/* Enable ADC peripheral (Power on)*/
	if(!(ADC1->CR2 & ADC_CR2_ADON)) {
		ADC1->CR2 |= ADC_CR2_ADON;
	}
}

void Adc_Disable(void)
{
	/* Power down ADC */
	ADC1->CR2 &= ~ADC_CR2_ADON;
}
