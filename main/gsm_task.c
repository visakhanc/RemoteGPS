/**
 * @file gsm_task.c
 * @author Visakhan
 * @date April 09, 2017
 * @details Originally created: July 5, 2015.
 * Modified for STM32F103 target
 */

#include <stdio.h>
#include "gsm_common.h"
#include "string.h"
#include "stm32f1_uart.h"
#include "num_utils.h"
#include "board.h"
#include "debug_console.h"


/**********************************************************
 * 					PUBLIC FUNCTIONS
 *********************************************************/

/**********************************************************
 * 					PRIVATE FUNCTIONS
 *********************************************************/
static void gsm_uart_rx_handler(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status);


/**********************************************************
 * 					GLOBAL VARIABLES
 *********************************************************/

/**
 * @brief Holds the status variables of GSM module
 */
gsm_status_struct gsm_status;

/**
 * @brief Buffer to receive response of HTTP GET or POST
 */
char http_buf[HTTP_BUF_SIZE];



/**********************************************************
 * 					PRIVATE VARIABLES
 *********************************************************/

/* FreeRTOS resources */
static TaskHandle_t xGsmTaskHandle;
static SemaphoreHandle_t xGsmTxSem;
static SemaphoreHandle_t xGsmTxSyncSem;
static EventGroupHandle_t xGsmEvent;

static uart_handle_t gsm_uart_handle;
static volatile bool http_buf_switch = false;
static uint8_t gsm_rx_isr_buf[2];
static uint8_t gsm_rx_buf[100];
static int gsm_rx_len;
static uart_config_t gsm_uart_config = {
								.BaudRate = GSM_UART_BAUDRATE,
								.ParityMode = Uart_Parity_None,
								.StopBits = Uart_StopBits_1,
								.EnableRx = true,
								.EnableTx = true
							};


#if GSM_DEBUG
/*
 *  Task for testing GSM module through a command prompt.
 *
 *  Console UART is used to receive command from a Test PC
 *  and the received command is sent to GSM module. The response
 *  from GSM module is printed out to the console
 *
 *  Note: Set GSM_DEBUG macro to 1 for enabling this task
 */

void gsm_debug_task(void *pArg)
{
	uint32_t len, i;
	char text[200];

	DEBUG_PUTS("DEBUG\r\n");

	while(1)
	{
		/* Get input from debug port */
		DEBUG_PUTS("\r\nM66>");
		i = 0;
		while(i < sizeof(text)) {
			text[i] = Debug_Console_GetChar();
			Debug_Console_PutChar(text[i]);
			if((text[i] == '\r') || (text[i] == '\n')) {
				text[i] = '\r';
				Debug_Console_PutChar('\n');
				break;
			}
			i++;
		}
		len = i+1;
		/*Send the command to GSM module */
		gsm_uart_acquire();
		gsm_uart_send(text, len);
		gsm_uart_release();
	}

}
#endif




/**
 * @brief Sends the GSM AT command through UART
 * @note The function sends CR-LF after the command string
 * @param cmd Null terminated AT command string
 * @return 0: Success, 1: Error
 */
int gsm_send_command(const char *cmd)
{
	int ret = 0;

	/* Send command */
	if(gsm_uart_send(cmd, strlen(cmd)) != 0) {
		ret = 1;
	}
	/* Send carriage return */
	if(gsm_uart_send("\r\n", 2) != 0) {
		ret = 1;
	}

	return ret;
}



/**
 * @brief Get access to GSM UART by acquiring semaphore
 * @note Tasks which communicate to GSM module would call this before
 * the section of code which deals with all communication with the module.
 * After this the Task releases the access by @ref gsm_uart_release()
 * @note The calling task would block indefinitely until acquiring the semaphore
 * @return
 *  	0 - Acquire successful
 *  	1 - Something went wrong
 */
int gsm_uart_acquire(void)
{
	/* Take Mutex for GSM UART Tx */
	if(pdTRUE != xSemaphoreTake(xGsmTxSem, portMAX_DELAY)) {
		return 1;
	}

	return 0;
}


/**
 * @brief Releases the access to GSM UART acquired by calling @ref gsm_uart_acquire()
 * @return
 *  	0 - Success
 *  	1 - Error
 */
int gsm_uart_release(void)
{
	/* Release Tx Mutex */
	if(pdTRUE != xSemaphoreGive(xGsmTxSem)) {
		return 1;
	}

	return 0;
}



/**
 * @brief Transmit data through GSM UART
 * @details This function:
 * 	1. Calls UART driver API
 * 	2. Waits until semaphore is released by UART Handler
 * @note Blocks waiting for semaphore to be released by UART handler
 * @param data Buffer containing data to be sent
 * @param len Number of bytes to send
 * @return Always returns 0
 */
int gsm_uart_send(const char *data, uint32_t len)
{
	/* Initiate UART driver to send the data */
	Uart_Send(GSM_UART, data, len);

	/* Block until UART handler signaling completion of Tx */
	xSemaphoreTake(xGsmTxSyncSem, portMAX_DELAY);

	return 0;
}




/**
 * @brief Send SMS message to a specified mobile number
 * @attention Message should be null terminated
 * @param buf Buffer containing message to be sent
 * @param length Number of characters to be sent
 * @param address Recipient address
 * @return 0: Success, 1: Error
 */
int gsm_send_sms(const char *buf, int length, const char *address)
{
	int ret = 0;
	uint8_t ctrl_z = 0x1A;
	uint8_t sms_buf[25];
	EventBits_t ev;

	/* Message content too long */
	if (length > 160) {
		DEBUG_PUTS("\nSMS too long");
		return 1;
	}

	if (buf[length] != '\0') {
		DEBUG_PUTS("\nSMS not proper string");
		return 1;
	}

	/* Set text mode */
	gsm_send_command("AT+CMGF=1");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 1000);
	if (ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\n\rerr:CMGF");
		return 1;
	}

	/* Set GSM character set */
	gsm_send_command("AT+CSCS=\"GSM\"");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 1000);
	if (ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\n\rerr:CSCS");
		return 1;
	}

	/* Check size of address */
	if (strlen(address) > 15) {
		DEBUG_PUTS("\naddress too long");
		return 1;
	}

	/* Message address */
// TODO: Implement printf
	//DEBUG_PRINTF((char *)sms_buf, sizeof(sms_buf) - 1, "AT+CMGS=\"%s\"", address);
	gsm_send_command((char*)sms_buf);
	vTaskDelay(1000); /* Wait for prompt '>' */

	/* Send message */
	if(gsm_uart_send(buf, length) != 0) {
		ret = 1;
	}

	if(gsm_uart_send((char *)&ctrl_z, 1) != 0) {
		ret = 1;
	}

	/* Wait for OK */
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 20 * 1000);
	if (ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\r\nERROR: SMS send");
		ret = 1;
	}
	else if(!(ev & EVENT_GSM_OK)) {
		DEBUG_PUTS("\r\nERROR: SMS timeout");
		ret = 1;
	}

	return ret;
}


/**
 * @brief Task for handling various response strings received from GSM module
 * @details This task waits for signal from UART Rx handler, indicating that a complete string is received
 *  The string is then checked against known response strings, relevant to the application
 *  For each of the identified string, corresponding event is signaled or global status is updated
 * @param pArg Not used
 */
void gsm_rx_task(void *pArg)
{
	uint8_t rx_buf[100];
	int len, temp;

	/**** Initialize GSM UART and associated FreeRTOS resources  ***/
	/* Create Mutex for GSM UART Tx */
	if((xGsmTxSem = xSemaphoreCreateMutex()) == NULL) {
		DEBUG_PUTS("ERROR: xGsmTxSem creation\r\n");
	}
	/* Create binary semaphore for GSM UART synchronization */
	if((xGsmTxSyncSem = xSemaphoreCreateBinary()) == NULL) {
		DEBUG_PUTS("ERROR: xGsmTxSyncSem creation\n\r");
	}
	/* Create Event for communicating various GSM responses */
	if((xGsmEvent = xEventGroupCreate()) == NULL) {
		DEBUG_PUTS("ERROR: xGsmEvent creation\n\r");
	}
	/* Initialize UART driver with given parameters */
	Uart_Init(GSM_UART, &gsm_uart_handle, &gsm_uart_config);
	/* Install our own handler for UART Rx data, which is called by the ISR */
	Uart_Set_Callback(GSM_UART, gsm_uart_rx_handler);
	/* Set Rx buffer for receiving data */
	Uart_Set_Rx_Params(GSM_UART, gsm_rx_isr_buf, 1);
	/* Enable UART Receive interrupt */
	Uart_StartReceive(GSM_UART);

	/* Use Task Notification to synchronize between UART Rx handler and GSM Rx task */
	xGsmTaskHandle = xTaskGetCurrentTaskHandle();

	while (1) {

		/* Wait for signal from UART Rx handler */
		if(ulTaskNotifyTake(pdTRUE, portMAX_DELAY) == 1) {
			gsm_status.power_state = GSM_POWERON;
			/* Copy GSM response string to local buffer */
			len = gsm_rx_len;
			if(len >= sizeof(rx_buf)) {
				continue;
			}
			memcpy((void *) rx_buf, (const void *) gsm_rx_buf, len);
			rx_buf[len] = '\0';

#if GSM_DEBUG
			DEBUG_PUTS("\r\n");
			Debug_Console_PutBuf(rx_buf, strlen(rx_buf));
#endif
			/* Response of 2 characters */
			if (2 == len) {
				if (0 == strcmp((char *)rx_buf, "OK")) {
					xEventGroupSetBits(xGsmEvent, EVENT_GSM_OK);
				}
			}
			/* Response of 4 characters */
			else if (4 == len) {
				if (0 == strcmp((char *)rx_buf, "RING")) {
					xEventGroupSetBits(xGsmEvent, EVENT_GSM_RING);
				}
			}
			/* Response of 5 characters */
			else if (5 == len) {
				if (0 == strcmp((char *)rx_buf, "ERROR")) {
					xEventGroupSetBits(xGsmEvent, EVENT_GSM_ERROR);
				}
			}
			/* Others */
			else {
				if (NULL != strstr((char *)rx_buf, "+CLIP")) {
					/* Get the caller number from CLIP line */
					len = get_quoted_string(&rx_buf[7], gsm_status.caller, sizeof(gsm_status.caller));
					if (len > 0) {
						xEventGroupSetBits(xGsmEvent, EVENT_GSM_CLIP);
					}
				}

				if (NULL != strstr((char *)rx_buf, "+CREG")) {
					/* Get and update registration status */
					if ((rx_buf[9] == '1') || (rx_buf[9] == '5')) {
						gsm_status.registered = true;
						xEventGroupSetBits(xGsmEvent, EVENT_GSM_CREG);
					}
					else {
						gsm_status.registered = false;
					}
				}

				if (NULL != strstr((char *)rx_buf, "CONNECT")) {
					if(gsm_status.http_sendto_module == true) {	/* http data to be sent to module */
						/* Notify that data can be sent now */
						xEventGroupSetBits(xGsmEvent, EVENT_GSM_HTTPCONNECT);
					}
					else { 	/* http data to be received from module */
#if GSM_DEBUG == 0
						/* Tell UART Rx handler to receive characters to http buffer */
						http_buf_switch = true;
#endif
					}
				}

				if (NULL != strstr((char *)rx_buf, "+CME ERROR")) {
					parse_decimal(&gsm_status.cme_error, (char *)&rx_buf[12], 4);
					xEventGroupSetBits(xGsmEvent, EVENT_GSM_CME_ERROR);
				}

			}

		}

	}

}



/**
 * @brief Set the given event(s) of the GSM event group
 * @param events Event(s) to be set. See @ref GSM_Event_Bits for possible events
 */
void gsm_set_event(uint32_t events)
{
	xEventGroupSetBits(xGsmEvent, events);
}


/**
 * @brief Clear the given event(s) of the GSM event group
 * @param events Event(s) to be cleared. See @ref GSM_Event_Bits for possible events
 */
void gsm_clear_event(uint32_t events)
{
	xEventGroupClearBits(xGsmEvent, events);
}


/**
 * @brief Wait for one or more GSM events until specified ticks is elapsed
 * @param events Event(s) to wait for; The function returns if any of the events is set
 * @param delay_ticks Number of ticks to wait (0: wait indefinitely)
 * @return The event which was set; 0 indicates no events were set before delay_ticks elapsed
 */
uint32_t gsm_wait_for_event(uint32_t events, uint32_t delay_ticks)
{
	TickType_t ticksToWait = (0 == delay_ticks)  ? portMAX_DELAY : delay_ticks;
	EventBits_t eventsToWait = (EventBits_t)events;

	return xEventGroupWaitBits(xGsmEvent, eventsToWait, pdTRUE, pdFALSE, ticksToWait);
}


/**
 * @brief UART ISR handler for GSM UART
 * @details Puts character into a buffer, until CR-LF termination is received and signals the @ref gsm_rx_task
 * @note This is called from the UART ISR
 * @param base UART base address
 * @param handle UART driver handle
 * @param status Status of UART when handler is called
 */
static void gsm_uart_rx_handler(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status)
{
	static int buf_n;
	static int http_n;
	static uint8_t prev_ch;
	static uint32_t depth;

	uint8_t ch;
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	/* Rx operation completed */
	if(Uart_Status_RxDone == status) {

		/*  The buffer pointer was incremented in UART ISR */
		handle->RxBuf--;
		/* Get received byte */
		ch = *(handle->RxBuf);
		/* reset receive size (decremented in UART ISR) */
		handle->RxSize = 1;

		if (false == http_buf_switch) {
			if(buf_n < sizeof(gsm_rx_buf)) {
				gsm_rx_buf[buf_n++] = ch;
				if (('\n' == ch) && ('\r' == prev_ch)) {
					gsm_rx_len = buf_n - 2;
					gsm_rx_buf[gsm_rx_len] = '\0';
					buf_n = 0;

					/* Signal only if more than CR-LF*/
					if (gsm_rx_len > 0) {
						vTaskNotifyGiveFromISR(xGsmTaskHandle, &xHigherPriorityTaskWoken);
					}
				}
				prev_ch = ch;
			}
		}
		else {
			/* Receive data into http buffer */
			if (http_n < sizeof(http_buf)) {
				http_buf[http_n++] = ch;
			}
			/* Check for end of HTTP READ ("OK\r\n") */
			if((depth == 0) && (ch == 'O')) {
				depth++;
			}
			else if((depth == 1) && (ch == 'K')) {
				depth++;
			}
			else if((depth == 2) && (ch == '\r')) {
				depth++;
			}
			else if((depth == 3) && (ch == '\n')) {
				depth++;
			}
			else {
				depth = 0;
			}
			if (depth == 4) {
				xEventGroupSetBitsFromISR(xGsmEvent, EVENT_GSM_HTTPREAD, &xHigherPriorityTaskWoken);
				http_buf_switch = false;
				http_buf[http_n - 4] = '\0';
				gsm_status.http_recv_len = http_n - 4;
				gsm_status.http_overflow = (http_n == sizeof(http_buf)) ? true : false;
				http_n = 0;
				depth = 0;
			}
		}
	}
	/* Tx operation completed */
	else if (Uart_Status_TxDone == status) {
		xSemaphoreGiveFromISR(xGsmTxSyncSem, &xHigherPriorityTaskWoken);
	}
	else {
		/* Error, should be Rx overrun */
	}

	/* If any RTOS calls caused a higher priority task to wake up, request for context switch */
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



