/**
 * 	@file	gps_task.c
 * 	@brief  Contains a Task for which processes received GPS sentences and updates GPS parameters
 * 			such as latitude, longitude etc.
 *	@author	Visakhan
 *	@date	Apr 09, 2017
 *
 *	Originally created: July 5, 2015
 * 	@note Modifications : Modified for STM32F103 target
 *
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


/**********************************************************
 * 					PUBLIC FUNCTIONS
 *********************************************************/

/**********************************************************
 * 					PRIVATE FUNCTIONS
 *********************************************************/
static void gps_uart_rx_handler(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status);


/**********************************************************
 * 					GLOBAL VARIABLES
 *********************************************************/

/* Total number of GPS sentences parsed by GPS task (Reset on each successful log) */
volatile int gps_count = 0;
/* GPS co-ordinates to be logged now IS DIFFERENT from previously logged co-ordinates? */
volatile bool gps_progress = false;


/**********************************************************
 * 					PRIVATE VARIABLES
 *********************************************************/
/* Buffer used by UART ISR to store received character from GPS module */
static uint8_t gps_rx_buf[2];

/* Buffer which holds an entire sentence of data from GPS module (excluding '$') */
static uint8_t gps_rx_sentence[200];

/* Number of characters stored in the sentence buffer */
static uint32_t gps_rx_len;

/* Task handle for GPS task */
static TaskHandle_t xGpsTaskHandle;

/* Semaphore which is released when UART transmission towards GPS
 * module is complete */
static xSemaphoreHandle xGpsTxSyncSem;

/* Semaphore which is released when positive Acknowledgment
 * is received from GPS module in response to a PMTK command */
static xSemaphoreHandle xGpsAckSem;

/* Structure holding updated GPS status values (latitude, longitude etc.) */
gps_info_struct gps_info;

/* Driver Handle structure for GPS UART */
uart_handle_t gps_uart_handle;

/* Configuration parameters for GPS UART */
uart_config_t gps_uart_config = {
						.BaudRate = GPS_UART_BAUDRATE,
						.ParityMode = Uart_Parity_None,
						.StopBits = Uart_StopBits_1,
						.EnableRx = true,
						.EnableTx = true
					};


/**
 * @brief 	Task to process received GPS sentences. Blocks while waiting until a complete sentence is
 * 		  	received, then process the sentences to update GPS status variables.
 * @param 	pArg : Not used
 * @retval 	None
 */
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
			/* PMTK acknowledgment (eg: $PMTK001,869,3*37<CR><LF> )*/
			else if(NULL != strstr(rx_sentence, "PMTK001")) {
				if(rx_sentence[12] == '3') {
					xSemaphoreGive(xGpsAckSem);
				}
			}
			else if(NULL != strstr(rx_sentence, "GPTXT")) {
				if(rx_sentence[25] == 'O') {
					if(rx_sentence[26] == 'K') { /* ANTSTATUS=OK */
						gps_info.ext_antenna = true;
					}
					else if(rx_sentence[26] == 'P') {
						gps_info.ext_antenna = false;
					}
				}
			}
			gps_count++;
		}
		/* Check for GPS Fix */
		if (GPS_NOFIX != gps_info.fix) {
			//LED_On();
		}
		else {
			//LED_Off();
		}
	}

}



uint32_t gps_send_cmd(uint8_t *cmdBuf, uint32_t len)
{
	uint32_t ret = 0;

	//Debug_Console_PutBuf(cmdBuf, len);
	/* Send command to GPS module */
	ret = gps_send(cmdBuf, len);
	/* Wait for GPS acknowledgment */
	if(xSemaphoreTake(xGpsAckSem, 2000) != pdTRUE) {
		DEBUG_PUTS("GPS_ACK_ERR\r\n");
		ret = 2;
	}
	return ret;
}


uint32_t gps_send(uint8_t *buf, uint32_t len)
{
	uint32_t ret = 0;

	/* Send command through UART */
	Uart_Send(GPS_UART, buf, len);
	/* Wait for Tx completion signal */
	if(xSemaphoreTake(xGpsTxSyncSem, 1000) != pdTRUE) {
		DEBUG_PUTS("GPS_TX_ERROR\r\n");
		ret = 1;
	}

	return ret;
}


/**
 * 	@brief 	Handler function for GPS UART Interrupt, called from within the UART ISR
 *			On getting first '$' character, puts the "subsequent" character into a buffer.
 * 			When CR-LF termination is received, signals the gps task.
 * 	@param 	base : UART Register structure base pointer
 * 	@param	handle : UART driver handle
 * 	@param	status : Status of UART operation
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

