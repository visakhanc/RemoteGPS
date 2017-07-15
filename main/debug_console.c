/*
 * debug_console.c
 *
 *  Created on: Apr 9, 2017
 *      Author: Visakhan C
 */


#include "debug_console.h"
#include "stm32f1_uart.h"
#include "circular_buf.h"
#include "FreeRTOS.h"
#include "semphr.h"

static void Debug_Uart_Callback(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status);


uint8_t __print_buf[100];
static uint8_t rx_isr_buf[2];
static uint8_t rx_buf[20];
static circbuf_t	rx_circ_buf;
static uart_config_t debug_uart_config = {
		.BaudRate = DEBUG_UART_BAUDRATE,
		.EnableRx = true,
		.EnableTx = true,
		.ParityMode = Uart_Parity_None,
		.StopBits = Uart_StopBits_1
};
static uart_handle_t debug_uart_handle;
static SemaphoreHandle_t xDebugRxSyncSem;
static SemaphoreHandle_t xDebugTxSyncSem;

uint32_t Debug_Console_Init(void)
{
	/* Create binary semaphore for UART synchronization */
	if((xDebugRxSyncSem = xSemaphoreCreateBinary()) == NULL) {
		return 1;
	}
	if((xDebugTxSyncSem = xSemaphoreCreateBinary()) == NULL) {
		return 1;
	}
	/* UART initialization */
	if(Uart_Init(DEBUG_UART, &debug_uart_handle, &debug_uart_config)) {
		return 1;
	}
	/* Initialize circular buffer for incoming characters */
	cb_init(&rx_circ_buf, rx_buf, sizeof(rx_buf), true);
	/* Set UART interrupt callback */
	Uart_Set_Callback(DEBUG_UART, Debug_Uart_Callback);
	/* Set location to store character from UART Rx interrupt */
	Uart_Set_Rx_Params(DEBUG_UART, rx_isr_buf, 1);
	/* Enable UART Rx interrupt to start receiving (Tx interrupt will be used only as needed) */
	Uart_StartReceive(DEBUG_UART);
	return 0;
}

/* Get a received character from Debug console
 * 	Returns:
 * 		Character received or 0 if empty
 */
uint8_t Debug_Console_GetChar(void)
{
	uint8_t ch = 0;
	uint32_t ret;

	/* Get character from circular buffer */
	ret = cb_read(&rx_circ_buf, &ch);
	if(ret != 0) {
		/* If buffer is empty, wait for a notification from Rx handler */
		xSemaphoreTake(xDebugRxSyncSem, portMAX_DELAY);
		/* Get character from buffer */
		cb_read(&rx_circ_buf, &ch);
	}
	return ch;
}

/* Writes a character to debug console */
uint32_t Debug_Console_PutChar(uint8_t ch)
{
	Uart_Send(DEBUG_UART, &ch, 1);
	/* Wait till transmit is finished */
	if(pdFALSE == xSemaphoreTake(xDebugTxSyncSem, 100)) {
		return 1;
	}
	return 0;
}

uint32_t Debug_Console_PutBuf(uint8_t *buf, uint32_t size)
{
	Uart_Send(DEBUG_UART, buf, size);
	/* Wait till transmit is finished */
	if(pdFALSE == xSemaphoreTake(xDebugTxSyncSem, 1000)) {
		return 1;
	}
	return 0;
}

/* Callback for Console UART Interrupt: Called from UART ISR; Puts received characters into a circular buffer
 *
 */
static void Debug_Uart_Callback(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(status == Uart_Status_TxDone) {
		/* Signal the task waiting for transmit to finish */
		xSemaphoreGiveFromISR(xDebugTxSyncSem, &xHigherPriorityTaskWoken);
	}
	if(status == Uart_Status_RxDone) {
		/* Decrement buffer pointer, which was incremented in ISR */
		handle->RxBuf--;
		/* Put received character to circular buffer */
		cb_write(&rx_circ_buf, *handle->RxBuf);
		/* Increment Rx size which was decremented in ISR (otherwise Rx interrupt will be disabled by ISR) */
		handle->RxSize++;
		/* If this is the first character since circular buffer was empty, signal any waiting task */
		if(cb_count(&rx_circ_buf) == 1) {
			xSemaphoreGiveFromISR(xDebugRxSyncSem, &xHigherPriorityTaskWoken);
		}
	}
	/* If xSemaphoreGiveFromISR caused a higher priority task to wake up, request for context switch */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);

}



