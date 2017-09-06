/**
 * @file	stm32f1_rcc.c
 * @brief	Driver for Reset & Clock Control module of STM32F103
 * @author	Visakhan
 * @date	June 8, 2017
 */

#include "stm32f1_rcc.h"
#include "system_stm32f1xx.h"


void RCC_HSE_OscInit(bool prediv)
{
	/* Turn on HSE clock */
	__HAL_RCC_HSE_CONFIG(RCC_HSE_ON);

	/* Wait until HSE clock ready */
	while((RCC->CR & RCC_CR_HSERDY) == RESET)
		;

	if(prediv) {
		__HAL_RCC_HSE_PREDIV_CONFIG(RCC_HSE_PREDIV_DIV2);
	}
}


void RCC_HSE_Disable(void)
{
	/* Turn off HSE clock */
	__HAL_RCC_HSE_CONFIG(RCC_HSE_OFF);

	/* Wait until HSE is disabled */
	while((RCC->CR & RCC_CR_HSERDY) == SET)
		;
}


void RCC_HSI_OscInit(void)
{
	/* Enable HSI Oscillator */

}


void RCC_HSI_Enable(void)
{
	/* Turn on HSI clock */
	__HAL_RCC_HSI_ENABLE();

	/* Wait until HSI is enabled */
	while((RCC->CR & RCC_CR_HSIRDY) == RESET)
		;
}

void RCC_HSI_Disable(void)
{
	/* Turn off HSI */
	__HAL_RCC_HSI_DISABLE();

	/* Wait until HSI is disabled */
	while((RCC->CR & RCC_CR_HSIRDY) == SET)
		;
}


void RCC_PLL_Init(uint32_t pll_src, uint32_t pll_mul)
{
	/* Disable PLL */
	__HAL_RCC_PLL_DISABLE();

	/* Wait until PLL is disabled */
	while((RCC->CR & RCC_CR_PLLRDY) == SET)
		;
	/* Setup PLL source and multiplication factor */
	__HAL_RCC_PLL_CONFIG(pll_src, pll_mul);

	/* Enable PLL */
	__HAL_RCC_PLL_ENABLE();


	/* Wait until PLL is ready */
	while((RCC->CR & RCC_CR_PLLRDY) == RESET)
		;
}


void RCC_Clock_Config(uint32_t sysclk_src)
{
	/* Set system clock source */
	__HAL_RCC_SYSCLK_CONFIG(sysclk_src);

	/* Wait until clock switches */
	while(__HAL_RCC_GET_SYSCLK_SOURCE() != (sysclk_src << 2))
		;

	/* Set HCLK:DIV/1 PCLK2:DIV/1 PCLK1: DIV/2 */
	RCC->CFGR |= RCC_CFGR_PPRE1_DIV2;

	/* Update SystemCoreClock global variable */
	SystemCoreClockUpdate();
}




uint32_t RCC_GetHCLKFreq(void)
{
	return SystemCoreClock;
}


uint32_t RCC_GetPCLK1Freq(void)
{
	uint32_t shift = (RCC->CFGR & RCC_CFGR_PPRE1) >> RCC_CFGR_PPRE1_Pos;
	if(shift > 3)
	  shift -= 3;
	return SystemCoreClock >> shift;
}


uint32_t RCC_GetPCLK2Freq(void)
{
	uint32_t shift = (RCC->CFGR & RCC_CFGR_PPRE2) >> RCC_CFGR_PPRE2_Pos;
	if(shift > 3)
	  shift -= 3;
	return SystemCoreClock >> shift;
}
