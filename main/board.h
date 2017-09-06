/**
 * @file
 * @brief  	Board initialization and utility functions
 * @date	April 4, 2017
 * @author	Visakhan
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/*------ Macros and definitions ------------*/

/*---------------- SYSTEM CONFIGURATION ------------------*/

/* Enable or disable battery charging indicator pin interrupt */
#define BATCHRG_INTERRUPT_ENABLED		1
/* Enable or disable GSM Netlight pin interrupt */
#define NETLIGHT_INTERRUPT_ENABLED		1
/* Enable USB */
#define USB_ENABLED						1

/* Define to 1 if GSM module need to be tested through command prompt */
#define GSM_DEBUG 						0
/* ENable Logging functionality */
#define LOG_ENABLED						1
/* Print GPS data in display task */
#define	DEBUG_GPS						0
/* Enable Sleep functionality to save power */
#define SLEEP_ENABLED					1

/* Default and minimum sleep interval while running on battery (in seconds) */
#define DEFAULT_SLEEP_INTERVAL			(2*60)
#define MINIMUM_SLEEP_INTERVAL			(2*60)
/* Default log interval (in seconds) */
#define DEFAULT_LOG_INTERVAL			8
#define MINIMUM_LOG_INTERVAL			8

/*------------------- SYSTEM CONFIGURATION --------------------*/



#define LED_PIN                         GPIO_PIN_4
#define LED_GPIO_PORT                   GPIOB

#define BUTTON_PIN                  	GPIO_PIN_0
#define BUTTON_GPIO_PORT            	GPIOA

#define GPS_1PPS_PIN					GPIO_PIN_5
#define GPS_1PPS_GPIO_PORT				GPIOA

#define GPS_FORCEON_PIN					GPIO_PIN_1
#define GPS_FORCEON_GPIO_PORT			GPIOA

#define GSM_DTR_PIN						GPIO_PIN_8
#define GSM_DTR_GPIO_PORT				GPIOA

#define GSM_PWRKEY_PIN					GPIO_PIN_0
#define GSM_PWRKEY_GPIO_PORT			GPIOB

#define GSM_NETLIGHT_PIN				GPIO_PIN_14
#define GSM_NETLIGHT_GPIO_PORT			GPIOB

#define BAT_MON_PIN						GPIO_PIN_1
#define BAT_MON_GPIO_PORT				GPIOB
#define BAT_MON_ADC_CHANNEL				9

#define BAT_CHRG_PIN					GPIO_PIN_3
#define BAT_CHRG_GPIO_PORT				GPIOB

#define OSC32_IN_PIN					GPIO_PIN_14
#define OSC32_IN_GPIO_PORT				GPIOC
#define OSC32_OUT_PIN					GPIO_PIN_15
#define OSC32_OUT_GPIO_PORT				GPIOC

#define USBDM_PIN						GPIO_PIN_11
#define USBDM_GPIO_PORT					GPIOA
#define USBDP_PIN						GPIO_PIN_12
#define USBDP_GPIO_PORT					GPIOA

#define USART3_TX_PIN					GPIO_PIN_10
#define USART3_RX_PIN					GPIO_PIN_11
#define	USART3_GPIO_PORT				GPIOB

#define USART2_TX_PIN					GPIO_PIN_2
#define USART2_RX_PIN					GPIO_PIN_3
#define	USART2_GPIO_PORT				GPIOA

#define USART1_TX_PIN					GPIO_PIN_9
#define USART1_RX_PIN					GPIO_PIN_10
#define	USART1_GPIO_PORT				GPIOA

#define DEBUG_UART						USART3
#define DEBUG_UART_BAUDRATE				115200

#define GPS_UART						USART2
#define GPS_UART_BAUDRATE				9600

#define GSM_UART						USART1
#define GSM_UART_BAUDRATE				9600


/* Public Function Declarations */

/**
 * @brief	Initializes GPIOs & external interrupts(EXTI) and enables all interrupts at NVIC level
 * @retval	None
 */
void Board_Init(void);

/**
  * @brief  Turns  LED On.
  * @retval None
  */
void LED_On(void);

/**
  * @brief  Turns LED Off.
  * @retval None
  */
void LED_Off(void);


/**
  * @brief  Toggles the LED.
  * @retval None
  */
void LED_Toggle(void);

/*!
 * @brief Get the state of the push button
 * @return Present state of the Push Button (1: Pressed, 0: Released)
 */
uint32_t Button_GetState(void);

/**
 * @brief  	Set the GPS FORCE_ON gpio pin to LOW state
 * @retval	None
 */
void GPS_ForceOn_Low(void);

/**
 * @brief  	Set the GPS FORCE_ON gpio pin to HIGH state
 * @retval  None
 */
void GPS_ForceOn_High(void);

/**
 * @brief 	Checks if battery is currently charging or not
 * @retval	TRUE if charging FALSE if not charging
 */
bool Battery_Charging(void);

/**
 * @brief 	Set GPIO pins in least power consuming state (Analog mode)
 * @retval 	None
 */
void GPIO_Set_LowPower(void);

/**
 * @brief 	Set GPIO pins in Normal configuration (To be called on returning from low power state)
 * @retval 	None
 */
void GPIO_Set_Normal(void);

/**
 * @brief	Reset MCU using Independent Watchdog (IWDG) after ~0.1ms delay
 * @retval	None
 */
void Watchdog_Reset(void);


#endif /* BOARD_H_ */
