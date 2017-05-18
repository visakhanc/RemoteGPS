/*
 * board.h
 *
 *  Created on: Apr 4, 2017
 *      Author: Visakhan C
 */

#ifndef BOARD_H_
#define BOARD_H_

#include "stm32f1xx_hal.h"
#include <stdbool.h>

/***** Macros and definitions *******/

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

/* Enable or disable battery charging indicator pin interrupt */
#define BATCHRG_INTERRUPT_ENABLED		0
/* Enable or disable GSM Netlight pin interrupt */
#define NETLIGHT_INTERRUPT_ENABLED		0

typedef struct _board_state {
	volatile bool charging;
	volatile float bat_voltage;
} board_state_t;


/* Globals */
extern board_state_t 	board_state;

/* Public Function Declarations */
void Board_Init(void);
void LED_On(void);
void LED_Off(void);
void LED_Toggle(void);
uint32_t Button_GetState(void);
void GPS_ForceOn_Low(void);
void GPS_ForceOn_High(void);
#endif /* BOARD_H_ */
