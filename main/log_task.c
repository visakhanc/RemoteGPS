/*
 * log_task.c
 *
 *	This task is responsible for, sending location data to a web server
 *	at regular intervals
 *
 *  Created on: Jun 17, 2016
 *      Author: Visakhan C
 */

#include <gps_common.h>
#include "config_private.h"
#include "gsm_common.h"
#include "http_utils.h"
#include "num_utils.h"
#include "debug_console.h"
#include "board.h"

/* Macros */
#define EVENT_START_LOGGING  	(1 << 0)
#define EVENT_STOP_LOGGING		(1 << 1)

/* Globals */
extern gps_info_struct gps_info;
extern volatile int gps_count;
extern volatile bool gps_progress;
extern uint8_t http_buf[];

static char log_post_params[128];
#define LOG_URL_OFFSET (sizeof(LOG_API_URL)-1)

static TaskHandle_t xLogTaskHandle;
volatile static bool logEnabled = false;

void log_task(void *pvParameters)
{
	int ret;
	EventBits_t ev;
	float cur_lat = 9.620093;
	float cur_lon = 76.618476;
	float prev_lat = 0.0;
	float prev_lon = 0.0;

	int offset;
	volatile int count = 0;
	uint32_t notifValue = 0;
	char print_buf[50];

	xLogTaskHandle = xTaskGetCurrentTaskHandle();
	do {
		/* Wait until GSM module get registered */
		vTaskDelay(1000);
	} while (gsm_status.registerd != true);

	vTaskDelay(3000);
	gsm_send_command("AT+QIFGCNT=0");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 1000);
	if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("Error: QIFGCNT");
	}
	gsm_send_command("AT+QICSGP=1,\"www\"");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 1000);
	if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("Error: QICSGP");
	}

	while(1)
	{

		/* Wait for Notification to start/stop logging */
		if(pdTRUE == xTaskNotifyWait(0, -1UL, &notifValue, 8000)) {
			if(notifValue & EVENT_START_LOGGING) {
				DEBUG_PUTS("\r\nLogging started");
				//if(gprs_configure() != 0) {
				//	DEBUG_PUTS("\r\nError: gprs_configure");
				//}
			}
			if(notifValue & EVENT_STOP_LOGGING) {
				//if(0 != gsm_uart_acquire()) { DEBUG_PUTS("\r\nlog: uart acquire fail"); }
				// TODO: disable logging on server too?
				//http_terminate();
				//http_close_context();
				//if(0 != gsm_uart_release()) { DEBUG_PUTS("\r\nlog: uart release fail"); }
				DEBUG_PUTS("\r\nLogging stopped");
			}
		}
#if LOG_ENABLED
		if((!count) || (true == logEnabled)) {

			/* Log only if position is reasonably accurate */
			if((gps_info.fix > NO_FIX) && (gps_info.hdop < 2.0)) {
				
				cur_lat = gps_info.latitude;
				cur_lon = gps_info.longitude;
				if((cur_lat != prev_lat) || (cur_lon != prev_lon)) {
					gps_progress = true;
				}
				offset = 0;
				offset += sprintf(&log_post_params[offset], "lat=");
				offset += float_to_string(&cur_lat, 6, (uint8_t *)&log_post_params[offset]);
				offset += sprintf(&log_post_params[offset], "&lon=");
				offset += float_to_string(&cur_lon, 6, (uint8_t *)&log_post_params[offset]);
				offset += sprintf(&log_post_params[offset], "&sp=%d&dr=%d&hd=%d", gps_info.velocity, gps_info.course, (int)(gps_info.hdop*10.0));
				offset += sprintf(&log_post_params[offset], "&dt=%d-%02d-%02d&tm=%02d:%02d:%02d",
								  gps_info.time.year+2000, gps_info.time.month, gps_info.time.date,
								  gps_info.time.hour, gps_info.time.min, gps_info.time.sec);
				offset += sprintf(&log_post_params[offset], "&cnt=%d&pr=%s&id=%d", gps_count,
								  	  	  (gps_progress == true) ? "true" : "false", count);


				//DEBUG_PUTS("\r\n%s", log_api_url);
				//DEBUG_PUTS("\r\nlen = %d", offset);
				
				/* acquire gsm uart */
				if(0 != gsm_uart_acquire()) { DEBUG_PUTS("\r\nlog: uart acquire fail"); }
				/* Send location to server using HTTP GET method */
				ret = http_post(LOG_API_URL, sizeof(LOG_API_URL)-1, (uint8_t *)log_post_params, offset);
				/* Release gsm uart */
				if(0 != gsm_uart_release()) { DEBUG_PUTS("\r\nlog: uart release fail"); }
				if(HTTP_OK == ret) {
					snprintf(print_buf, sizeof(print_buf), "\r\nPOST success (response: %d B )", gsm_status.http_recv_len);
					Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
					Debug_Console_PutBuf(http_buf, strlen(http_buf));
					LED_On();
					vTaskDelay(100);
					LED_Off();
					/* Check whether to continue logging */
					if(strstr((char *)http_buf, "Cont") != NULL) {
						DEBUG_PUTS("Continuing Log..\r\n");
						logEnabled = true;
					}
					else {
						logEnabled = false;
						DEBUG_PUTS("Stopping Log..\r\n");
						//http_terminate();
						//http_close_context();
					}
					count++;
				}
				else {
					sprintf(print_buf, "\r\nlog: POST failed (%d) (url: %d bytes)\r\n", ret, offset);
					Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
					Debug_Console_PutBuf((uint8_t *)log_post_params, strlen(log_post_params));
					Debug_Console_PutBuf(http_buf, gsm_status.http_recv_len);
					//http_terminate(); // close http stack
				}

				prev_lat = cur_lat;
				prev_lon = cur_lon;
				gps_count = 0;
				gps_progress = false;


			}
		}
#endif
	} /* while(1) */
}


/* Called externally to start/stop logging */
void log_task_switch(void)
{
	uint32_t	notifValue;

	if(true == logEnabled) {
		logEnabled = false;
		/* Set task notification value to stop logging */
		notifValue = EVENT_STOP_LOGGING;
	}
	else {
		logEnabled = true;
		/* Set task notification value to start logging */
		notifValue = EVENT_START_LOGGING;
	}

	xTaskNotify(xLogTaskHandle, notifValue, eSetBits);
}
