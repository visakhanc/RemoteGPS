/**
 * @file
 * @brief  	Board initialization and utility functions
 * @date	April 4, 2017
 * @author	Visakhan
 */

#include "board.h"
//#include "stm32f1xx_hal.h"

/**
 * @brief This structure contains initialization settings for each port pins used
 */
struct gpio_init_entry {
	GPIO_TypeDef *Port;		/**< Pointer to Register structure of GPIO port which contains the pin */
	GPIO_InitTypeDef Init;	/**< Structure containing initialization settings for the pin */
} Gpio_Init_Table[] = {
{LED_GPIO_PORT, 		{LED_PIN, 			GPIO_MODE_OUTPUT_PP, 	GPIO_NOPULL, 	GPIO_SPEED_HIGH }		},
{BUTTON_GPIO_PORT, 		{BUTTON_PIN, 		GPIO_MODE_INPUT, 		GPIO_PULLDOWN,	GPIO_SPEED_MEDIUM }		},
{USART3_GPIO_PORT, 		{USART3_TX_PIN, 	GPIO_MODE_AF_PP,		GPIO_NOPULL, 	GPIO_SPEED_HIGH}		},
{USART3_GPIO_PORT, 		{USART3_RX_PIN, 	GPIO_MODE_INPUT,		GPIO_PULLUP, 	GPIO_SPEED_HIGH}		},
{USART2_GPIO_PORT, 		{USART2_TX_PIN, 	GPIO_MODE_AF_PP,		GPIO_NOPULL, 	GPIO_SPEED_HIGH}		},
{USART2_GPIO_PORT, 		{USART2_RX_PIN, 	GPIO_MODE_INPUT,		GPIO_PULLUP, 	GPIO_SPEED_HIGH}		},
{USART1_GPIO_PORT, 		{USART1_TX_PIN, 	GPIO_MODE_AF_PP,		GPIO_NOPULL, 	GPIO_SPEED_HIGH}		},
{USART1_GPIO_PORT, 		{USART1_RX_PIN, 	GPIO_MODE_INPUT,		GPIO_PULLUP, 	GPIO_SPEED_HIGH}		},
{GPS_1PPS_GPIO_PORT, 	{GPS_1PPS_PIN,		GPIO_MODE_INPUT,		GPIO_NOPULL,	GPIO_SPEED_MEDIUM }		},
{GPS_FORCEON_GPIO_PORT,	{GPS_FORCEON_PIN, 	GPIO_MODE_OUTPUT_PP,	GPIO_NOPULL,	GPIO_SPEED_MEDIUM }		},
{GSM_DTR_GPIO_PORT,		{GSM_DTR_PIN,		GPIO_MODE_OUTPUT_PP,	GPIO_NOPULL,	GPIO_SPEED_MEDIUM }		},
{GSM_PWRKEY_GPIO_PORT,	{GSM_PWRKEY_PIN,	GPIO_MODE_OUTPUT_PP,	GPIO_NOPULL,	GPIO_SPEED_MEDIUM }		},
{GSM_NETLIGHT_GPIO_PORT,{GSM_NETLIGHT_PIN,	GPIO_MODE_INPUT,		GPIO_NOPULL,	GPIO_SPEED_HIGH }		},
{BAT_MON_GPIO_PORT,		{BAT_MON_PIN,		GPIO_MODE_ANALOG,		GPIO_NOPULL,	GPIO_SPEED_MEDIUM}		},
{BAT_CHRG_GPIO_PORT,	{BAT_CHRG_PIN,		GPIO_MODE_INPUT,		GPIO_NOPULL,	GPIO_SPEED_MEDIUM}		},
{USBDP_GPIO_PORT, 		{USBDP_PIN, 		GPIO_MODE_AF_INPUT,		GPIO_NOPULL, 	GPIO_SPEED_HIGH}		},
{USBDM_GPIO_PORT, 		{USBDM_PIN, 		GPIO_MODE_AF_INPUT,		GPIO_NOPULL,	GPIO_SPEED_HIGH}		},
{OSC32_IN_GPIO_PORT,	{OSC32_IN_PIN, 		GPIO_MODE_AF_INPUT,		GPIO_NOPULL, 	GPIO_SPEED_HIGH}		},
{OSC32_OUT_GPIO_PORT,	{OSC32_OUT_PIN, 	GPIO_MODE_AF_INPUT,		GPIO_NOPULL,	GPIO_SPEED_HIGH}		}

};


board_state_t 	board_state;


void Board_Init(void)
{
	uint32_t 	index;

	/* Enable the Peripheral clocks */
	__HAL_RCC_AFIO_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_USART3_CLK_ENABLE();
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_USART1_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	__HAL_RCC_ADC_CONFIG(RCC_ADCPCLK2_DIV8);  /* Set prescaler for ADC clock (ADC clock should be <14MHz) */
	__HAL_RCC_TIM2_CLK_ENABLE();

	/* Release JTAG Reset pin for PB3 & PB4 GPIO functionality */
	__HAL_AFIO_REMAP_SWJ_NOJTAG();

	for(index = 0; index < sizeof(Gpio_Init_Table)/sizeof(Gpio_Init_Table[0]); index++)
	{
		HAL_GPIO_Init(Gpio_Init_Table[index].Port, &Gpio_Init_Table[index].Init);
	}

	/* Set Interrupt priorities in NVIC (should be lesser (higher number) than SVC Interrupt configured for FreeRTOS) */
	HAL_NVIC_SetPriority(USART1_IRQn, 8, 0);
	HAL_NVIC_SetPriority(USART2_IRQn, 8, 0);
	HAL_NVIC_SetPriority(USART3_IRQn, 8, 0);
	HAL_NVIC_SetPriority(EXTI0_IRQn, 8, 0);
	HAL_NVIC_SetPriority(EXTI3_IRQn, 8, 0);
	HAL_NVIC_SetPriority(EXTI15_10_IRQn, 8, 0);
	HAL_NVIC_SetPriority(RTC_IRQn, 8, 0);
	HAL_NVIC_SetPriority(RTC_Alarm_IRQn, 8, 0);

	/* Enable required interrupts in NVIC */
	NVIC_EnableIRQ(USART1_IRQn);
	NVIC_EnableIRQ(USART2_IRQn);
	NVIC_EnableIRQ(USART3_IRQn);
	NVIC_EnableIRQ(EXTI0_IRQn);
	NVIC_EnableIRQ(EXTI3_IRQn);
	NVIC_EnableIRQ(EXTI15_10_IRQn);
	NVIC_EnableIRQ(RTC_IRQn);
	NVIC_EnableIRQ(RTC_Alarm_IRQn);
	NVIC_EnableIRQ(TIM2_IRQn);

	//NVIC_EnableIRQ(RCC_IRQn);
	//RCC->CIR = RCC_CIR_LSERDYIE;

	/* Initial values for output pins */
	HAL_GPIO_WritePin(LED_GPIO_PORT,LED_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GSM_DTR_GPIO_PORT, GSM_DTR_PIN, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_PORT, GSM_PWRKEY_PIN, GPIO_PIN_RESET);

	/* Setup External interrupt for button pin */
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI0_PA; 	/* PA0 pin to EXTI0 */
	EXTI->RTSR |= EXTI_RTSR_TR0;  	/* Rising edge trigger */
	EXTI->IMR |= EXTI_IMR_IM0;		/* Enable EXTI0 interrupt */

#if BATCHRG_INTERRUPT_ENABLED
	/* Setup external interrupt for battery charging indicator pin */
	AFIO->EXTICR[0] |= AFIO_EXTICR1_EXTI3_PB;
	EXTI->RTSR |= EXTI_RTSR_RT3; /* Rising and Falling edge trigger */
	EXTI->FTSR |= EXTI_FTSR_FT3;
	EXTI->IMR |= EXTI_IMR_IM3;
#endif

#if NETLIGHT_INTERRUPT_ENABLED
	/* Setup external interrupt for NETLIGHT pin */
	AFIO->EXTICR[3] |= AFIO_EXTICR4_EXTI14_PB;
	EXTI->RTSR |= EXTI_RTSR_RT14;	/* Rising edge trigger */
	//EXTI->FTSR |= EXTI_FTSR_FT14; /* Falling edge trigger */
	EXTI->IMR |= EXTI_IMR_IM14;
#endif

	/* Setup EXTI 17 (RTC) for rising edge interrupt */
	EXTI->RTSR |= EXTI_RTSR_RT17;
	EXTI->IMR |= EXTI_IMR_IM17;
}


uint32_t Button_GetState(void)
{
	return HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN);
}



void LED_On(void)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_SET);
}


void LED_Off(void)
{
  HAL_GPIO_WritePin(LED_GPIO_PORT, LED_PIN, GPIO_PIN_RESET);
}

void LED_Toggle(void)
{
  HAL_GPIO_TogglePin(LED_GPIO_PORT, LED_PIN);
}


void GPS_ForceOn_Low(void)
{
	HAL_GPIO_WritePin(GPS_FORCEON_GPIO_PORT, GPS_FORCEON_PIN, GPIO_PIN_RESET);
}

void GPS_ForceOn_High(void)
{
	HAL_GPIO_WritePin(GPS_FORCEON_GPIO_PORT, GPS_FORCEON_PIN, GPIO_PIN_SET);
}



