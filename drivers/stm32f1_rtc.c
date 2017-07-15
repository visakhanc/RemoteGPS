/**
 * @file	stm32f1_rtc.c
 * @brief	Driver to access RTC peripheral on STM32F1 device
 * @author	Visakhan
 * @date	May 25, 2017
 */

#include "stm32f1xx_hal.h"
#include "stm32f1_rtc.h"

static void RTC_Enable_Config(void);
static void RTC_Disable_Config(void);


/**
 * @brief Initialize RTC clock and set LSE (32kHz) as the clock
 */
void RTC_Init(void)
{
	__HAL_RCC_PWR_CLK_ENABLE();
	/* Disable backup register protection - to configure RTC */
	PWR->CR |= PWR_CR_DBP;

	/* Configure RTC clock only if RTC not enabled now (means backup power lost) */
	if(!(RCC->BDCR & RCC_BDCR_RTCEN)) {
		/* If RTC clock is already selected, it can be changed only with a Backup Reset */
		//if(RCC->BDCR & RCC_BDCR_RTCSEL) {
		//__HAL_RCC_BACKUPRESET_FORCE();	// Reset backup domain
		//__HAL_RCC_BACKUPRESET_RELEASE();
		//}
		/* Turn on LSE */
		RCC->BDCR |= RCC_BDCR_LSEON;
		while(!(RCC->BDCR & RCC_BDCR_LSERDY))
			;
		/* Change RTC clock selection to LSE, Enable RTC clock */
		RCC->BDCR |= RCC_BDCR_RTCSEL_LSE | RCC_BDCR_RTCEN;
	}

	RTC_Enable_Config();
	RTC->PRLH = 0;
	RTC->PRLL = 0x7FFF;
	//RTC->CRH |= RTC_CRH_SECIE;
	RTC_Disable_Config();
}

/**
 * @brief Program RTC counter with given value
 * @param count Count to be set for RTC counter
 */
void RTC_Set_Counter(uint32_t count)
{
	RTC_Enable_Config();
	/* Set count register */
	RTC->CNTH = count >> 16;
	RTC->CNTL = count & 0xFFFF;
	RTC_Disable_Config();
}


/**
 * @brief Get the current RTC counter
 * @retval Current RTC count value
 */
uint32_t RTC_Get_Count(void)
{
	return (RTC->CNTL & 0xFFFF) + (RTC->CNTH << 16);
}

/**
 * @brief Configure Alarm register of RTC
 * @param val Alarm register value to be set
 */
void RTC_Alarm_Config(uint32_t val)
{
	RTC_Enable_Config();
	/* Set alarm register value */
	RTC->ALRH = val >> 16;
	RTC->ALRL = val & 0xFFFF;
	/* Clear alarm flag */
	RTC->CRL &= ~RTC_CRL_ALRF;
	/* Enable RTC alarm interrupt */
	RTC->CRH |= RTC_CRH_ALRIE;
	RTC_Disable_Config();
}


static void RTC_Enable_Config(void)
{
	/* Wait until write is finished */
	while(!(RTC->CRL & RTC_CRL_RTOFF))
		;
	/* Enter configuration mode */
	RTC->CRL |= RTC_CRL_CNF;
}


static void RTC_Disable_Config(void)
{
	/* Exit config mode */
	RTC->CRL &= ~RTC_CRL_CNF;
	/* Wait until write is finished */
	while(!(RTC->CRL & RTC_CRL_RTOFF))
		;
}
