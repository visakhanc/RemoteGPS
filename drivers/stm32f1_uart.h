/*
 * stm32f1_uart.h
 *
 *	Interrupt driven driver for STM32F1xx UART module
 *
 *  Created on: Apr 8, 2017
 *      Author: Visakhan C
 */

#ifndef STM32F1_UART_H_
#define STM32F1_UART_H_

#include "stm32f1xx.h"
#include <stdbool.h>

#define UART_PARITY_NONE                    ((uint32_t)0x00000000)
#define UART_PARITY_EVEN                    ((uint32_t)USART_CR1_PCE)
#define UART_PARITY_ODD                     ((uint32_t)(USART_CR1_PCE | USART_CR1_PS))

#define UART_STOPBITS_1                     ((uint32_t)0x00000000)
#define UART_STOPBITS_2                     ((uint32_t)USART_CR2_STOP_1)


#define UART_DIV_SAMPLING16(_PCLK_, _BAUD_)         (((_PCLK_)*25)/(4*(_BAUD_)))
#define UART_DIVMANT_SAMPLING16(_PCLK_, _BAUD_)     (UART_DIV_SAMPLING16((_PCLK_), (_BAUD_))/100)
#define UART_DIVFRAQ_SAMPLING16(_PCLK_, _BAUD_)     (((UART_DIV_SAMPLING16((_PCLK_), (_BAUD_)) - (UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) * 100)) * 16 + 50) / 100)
/* UART BRR = mantissa + overflow + fraction
            = (UART DIVMANT << 4) + (UART DIVFRAQ & 0xF0) + (UART DIVFRAQ & 0x0F) */
#define UART_BRR_SAMPLING16(_PCLK_, _BAUD_)            (((UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) << 4) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0xF0)) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0x0F))

typedef enum {
	Uart_Parity_None = UART_PARITY_NONE,
	Uart_Parity_Odd = UART_PARITY_ODD,
	Uart_Parity_Even = UART_PARITY_EVEN
} uart_parity_mode_t;

typedef enum {
	Uart_StopBits_1 = UART_STOPBITS_1,
	Uart_StopBits_2 = UART_STOPBITS_2
} uart_stopbit_mode_t;

typedef enum {
	Uart_Status_TxDone = 0,
	Uart_Status_RxDone,
	Uart_Status_Error
} uart_status_t;

typedef enum {
	Uart_Tx_Ready = 0,
	Uart_Tx_Busy,
	Uart_Tx_Error
} uart_tx_status_t;

typedef enum {
	Uart_Rx_Ready = 0,
	Uart_Rx_Busy,
	Uart_Rx_Error
} uart_rx_status_t;

typedef struct uart_handle uart_handle_t;

typedef void (*uart_callback_t)(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status);

struct uart_handle {
	volatile bool initialized;
	volatile uint8_t *RxBuf;
	volatile uint32_t RxSize;
	volatile uint8_t *TxBuf;
	volatile uint32_t TxSize;
	uart_callback_t callback;
	volatile uart_tx_status_t TxState;
	volatile uart_rx_status_t RxState;
};

typedef struct {
	uint32_t BaudRate;
	uart_parity_mode_t ParityMode;
	uart_stopbit_mode_t StopBits;
	bool EnableTx;
	bool EnableRx;
} uart_config_t;


uint32_t Uart_Init(USART_TypeDef *base, uart_handle_t *handle, uart_config_t *config);
uint32_t Uart_Set_Callback(USART_TypeDef *base, uart_callback_t callback);
uint32_t Uart_Send(USART_TypeDef *base, uint8_t *data, uint32_t size);
uint32_t Uart_StartReceive(USART_TypeDef *base);
uint32_t Uart_StopReceive(USART_TypeDef *base);
uint32_t Uart_Set_Rx_Params(USART_TypeDef *base, uint8_t *buf, uint32_t size);
uint32_t Uart_Receive(USART_TypeDef *base, uint8_t *buf, uint32_t size);
uint32_t Uart_DisableRx(USART_TypeDef *base);
uint32_t Uart_EnableRx(USART_TypeDef *base);

#endif /* STM32F1_UART_H_ */
