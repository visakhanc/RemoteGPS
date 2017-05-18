/*
 * gps_task.c
 *
 *  Created on: Jul 5, 2015
 *  Modified: Apr 09, 2017
 *  	- Modified for STM32F103 target
 *
 *  Author: Visakhan C
 */

#include <gps_common.h>
#include <stdio.h>
#include "string.h"
#include "stm32f1_uart.h"
#include "board.h"
#include "debug_console.h"
#include "num_utils.h"

/* FreeRTOS */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

uint32_t gps_send_cmd(uint8_t *cmdBuf, uint32_t len);
static void gps_uart_rx_handler(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status);

static uint8_t gps_rx_buf[2], gps_rx_sentence[200];
static uint32_t gps_rx_len;
static TaskHandle_t xGpsTaskHandle;
static xSemaphoreHandle xGpsTxSyncSem;
static xSemaphoreHandle xGpsAckSem;
gps_info_struct gps_info;
uart_handle_t gps_uart_handle;
uart_config_t gps_uart_config = {
						.BaudRate = GPS_UART_BAUDRATE,
						.ParityMode = Uart_Parity_None,
						.StopBits = Uart_StopBits_1,
						.EnableRx = true,
						.EnableTx = true
					};

volatile int gps_count = 0;
volatile bool gps_progress = false;

/*TASK*-----------------------------------------------------
 *
 * Task Name    : gps_task
 * Comments     :
 *
 *
 *END*-----------------------------------------------------*/

void gps_task(void *pArg)
{

	char rx_sentence[200];
	char *data_ptr;
	uint32_t len;
	uint32_t notificationValue;

	/* Set FORCEON signal to high at startup */
	GPS_ForceOn_High();

	/* Use Task Notification to synchronize between UART Rx handler and GPS task */
	xGpsTaskHandle = xTaskGetCurrentTaskHandle();

	xGpsTxSyncSem = xSemaphoreCreateBinary();
	xGpsAckSem = xSemaphoreCreateBinary();

	/* Initialize UART driver with given parameters */
	Uart_Init(GPS_UART, &gps_uart_handle, &gps_uart_config);

	Uart_Set_Callback(GPS_UART, gps_uart_rx_handler);

	/* Set receive buffer pointer and size */
	Uart_Set_Rx_Params(GPS_UART, gps_rx_buf, 1);

	/* Enable RX interrupt (start reception of bytes from GPS module) */
	Uart_StartReceive(GPS_UART);

	while (1) {
		/* Wait for notification from UART Rx handler */
		if((notificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) == 1) {
			/* Copy the received sentence */
			len = gps_rx_len;
			memcpy((void *) rx_sentence, (void *) gps_rx_sentence, len);

			/* Parse the received sentence to update gps_info structure */
			/* GGA sentence */
			if (NULL != strstr(rx_sentence, "GGA")) {
				data_ptr = &rx_sentence[6];
				gps_parse_gga(data_ptr, &gps_info);
			}
			/* RMC sentence */
			else if (NULL != strstr(rx_sentence, "RMC")) {
				data_ptr = &rx_sentence[6];
				gps_parse_rmc(data_ptr, &gps_info);
			}
			/* PMTK acknowledgment */
			else if(NULL != strstr(rx_sentence, "PMTK001")) {
				if(rx_sentence[12] == '3') {
					xSemaphoreGive(xGpsAckSem);
				}
			}
			else if(NULL != strstr(rx_sentence, "PQTXT")) {
				if(rx_sentence[8] == 'O') { /* ..OK.. */
					xSemaphoreGive(xGpsAckSem);
				}
			}
			gps_count++;
		}
		/* Check for GPS Fix */
		if (NO_FIX != gps_info.fix) {
			//LED_On();
		}
		else {
			//LED_Off();
		}
	}

}


/*
 * 	Send PMTK command to GPS module
 * 	Parameters:
 * 		cmdBuf: Command string in the format: "$PMTK...*"
* 		len: number of characters in the command string
 */
uint32_t gps_send_cmd(uint8_t *cmdBuf, uint32_t len)
{
	uint32_t ret = 0;
	uint8_t checksum = 0;
	uint32_t index = 1;
	while(index < len-1)
	{
		checksum ^= cmdBuf[index++];
	}
	index++; // skip '*'
	byte_to_str(checksum, &cmdBuf[index]);
	index +=2;
	cmdBuf[index++] = '\r';
	cmdBuf[index] = '\n';
	Debug_Console_PutBuf(cmdBuf, index+1);
	/* Send command through UART */
	Uart_Send(GPS_UART, cmdBuf, index+1);
	/* Wait for Tx completion signal */
	if(xSemaphoreTake(xGpsTxSyncSem, 1000) != pdTRUE) {
		DEBUG_PUTS("GPS_TX_ERROR\r\n");
		ret = 1;
	}
	/* Wait for GPS acknowledgment */
	if(xSemaphoreTake(xGpsAckSem, 2000) != pdTRUE) {
		DEBUG_PUTS("GPS_ACK_ERR\r\n");
		ret = 2;
	}
	return ret;
}


/*
 * 	UART handler for GPS UART
 *	(Called from within the UART ISR)
 *
 * 	On getting first '$' character, puts the subsequent character into a buffer.
 * 	When CR-LF termination is received, signals the gps task.
 */
static void gps_uart_rx_handler(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status)
{
	static uint32_t buf_n;
	static uint8_t prev_ch;
	uint8_t ch;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(Uart_Status_RxDone == status) {
		/*  The buffer pointer was incremented in UART ISR */
		handle->RxBuf--;
		/* Get received byte */
		ch = *(handle->RxBuf);
		/* Reset Rx size (decremented in UART ISR), otherwise UART driver will disable Rx interrupt! */
		handle->RxSize = 1;

		if ('$' == ch) {
			buf_n = 0;
		}
		else {
			if (buf_n < sizeof(gps_rx_sentence)) {
				gps_rx_sentence[buf_n++] = ch;
				if (('\n' == ch) && (prev_ch == '\r')) {
					gps_rx_len = buf_n;
					vTaskNotifyGiveFromISR(xGpsTaskHandle, &xHigherPriorityTaskWoken);
				}
			}
		}
		prev_ch = ch;
	}
	else if(Uart_Status_TxDone == status) {
		xSemaphoreGiveFromISR(xGpsTxSyncSem, &xHigherPriorityTaskWoken);
	}
	else {
		/* Uart Error: should be overrun */
	}

	/* Request context switch if any RTOS calls caused a higher priority task ready */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

