/*
 * message_task.c
 *
 *	This task is responsible for reporting details of current location by
 *	sending SMS message to the user, whenever requested
 *
 *  Created on: Jun 17, 2016
 *      Author: Visakhan C
 */

#include <gps_common.h>
#include "board.h"
#include "config_private.h"
#include "gsm_common.h"
#include "http_utils.h"
#include "num_utils.h"
#include "debug_console.h"
#include "stm32f1_adc.h"


#if 0
static int message_update_gpsdata(void);
static int message_update_location(int offset);
static int message_update_error(const char *err, int len);
static void print_tx_data(void);

/* Globals */
extern gps_info_struct gps_info;
extern gsm_status_struct gsm_status;

static float cur_lat;
static float cur_lon;
static char page_buf[256];
static uint8_t gsm_tx_buf[200];

#endif


//static char maps_api_url[100] = MAPS_API_URL;
#define URL_OFFSET  (sizeof(MAPS_API_URL)-1)

//static char print_buf[50];




void message_task(void *pArg)
{
	//char gsm_cmd[20];
	//EventBits_t ev;
	//uint32_t count;

#if !GSM_DEBUG
	DEBUG_PUTS("Initializing Modem\r\n");
	gsm_start();
#endif

	while (1) {
		vTaskDelay(2000);
#if 0
		/* Waiting for the call */
		ev = gsm_wait_for_event(EVENT_GSM_SENDMSG, 0);
		if(ev & EVENT_GSM_SENDMSG) {

			DEBUG_PUTS("\r\nCall from ");
			Debug_Console_PutBuf(gsm_status.caller, strlen((char *)gsm_status.caller));
			DEBUG_PUTS("\r\n");

			gsm_uart_acquire();
			gsm_send_command("ATH");
			gsm_uart_release();

			/* Get access to GSM UART */
			if(gsm_uart_acquire() != 0) {
				DEBUG_PUTS("\r\nmessage: gsm_uart_acquire() failed");
				/* Wait again for call */
				continue;
			}

			cur_lat = gps_info.latitude;
			cur_lon = gps_info.longitude;
			len = float_to_string(&cur_lat, 6, (uint8_t *)&maps_api_url[URL_OFFSET]);
			maps_api_url[URL_OFFSET + len] = ',';
			len++;
			float_to_string(&cur_lon, 6, (uint8_t *)&maps_api_url[URL_OFFSET + len]);

			/* Update message text with gps data */
			len = message_update_gpsdata();

			/* Lookup reverse geocoding url only if resonably accurate */
			if ((gps_info.fix > NO_FIX) && (gps_info.hdop < 2.0)) {
				if (0 == http_open_context()) {
					if (0 == http_init()) {
						if (0 == http_get(maps_api_url)) {
							DEBUG_PUTS("\nget success: %d\r\n", gsm_status.http_recv_len);
							if (0 == http_find_string("formatted_address", (uint8_t *)page_buf, sizeof(page_buf))) {
								DEBUG_PUTS("\nFound :)\n\r");
								len = message_update_location(len);
								DEBUG_PUTS("%s", gsm_tx_buf);
								if (0 != gsm_send_sms((char *)gsm_tx_buf, len, (char *)gsm_status.caller)) {
									DEBUG_PUTS("\nSMS failed\r\n");
								}
							}
							else {
								DEBUG_PUTS("\nNot found :(\n\r");
								DEBUG_PUTS("\n%s\r\n", maps_api_url);
							}
						}
						else {
							DEBUG_PUTS("\nLookup failed\n\r");
							/* update error detail in message and send message */
							len = message_update_error("Lookup failed\r\n", len);
							if (0 != gsm_send_sms((char *)gsm_tx_buf, len, (char *)gsm_status.caller)) {
								DEBUG_PUTS("\nSMS failed!\r\n");
							}
						}
						http_terminate();
					}
					http_close_context();
				}
			}
			else {
				DEBUG_PUTS("\nSending message to %s...\n\r", gsm_status.caller);
				if (0 != gsm_send_sms((char *)gsm_tx_buf, len, (char *)gsm_status.caller)) {
					DEBUG_PUTS("\nSMS failed\r\n");
				}
			}
			/* Release GSM UART */
			gsm_uart_release();
		}
#endif
	}

}


#if 0

static int message_update_gpsdata(void)
{
	int offset = 0;
	uint8_t str[16];

	offset = sprintf((char *)&gsm_tx_buf[offset], "LatLon:");
	float_to_string(&cur_lat, 6, str);
	offset += sprintf((char *)&gsm_tx_buf[offset], "%s,", str);
	float_to_string(&cur_lon, 6, str);
	offset += sprintf((char *)&gsm_tx_buf[offset], "%s\r\n", str);
	offset += sprintf((char *)&gsm_tx_buf[offset], "Dir:%d deg\r\n", gps_info.course);
	offset += sprintf((char *)&gsm_tx_buf[offset], "Speed:%d kph\r\n", gps_info.velocity);
	offset += sprintf((char *)&gsm_tx_buf[offset], "HDOP:%d\r\n", (int) (gps_info.hdop * 10.0));

	return offset;
}

static int message_update_location(int offset)
{
	int components = 0;
	char *ptr = page_buf;

	while (*ptr != ':')
		ptr++;

	ptr += 3;
	while (components < 5) {
		gsm_tx_buf[offset++] = *ptr++;
		if (',' == *ptr) {
			components++;
		}

		if ('\"' == *ptr) {
			break;
		}
	}

	gsm_tx_buf[offset] = '\0';

	return offset;
}


static int message_update_error(const char *err, int len)
{
	int offset = len;
	offset += sprintf((char *)&gsm_tx_buf[len], "%s", err);
	return offset;
}


static void print_tx_data(void)
{
	DEBUG_PUTS("\n\r");
	Debug_Console_PutBuf(gsm_tx_buf, strlen(gsm_tx_buf));
	DEBUG_PUTS("\n\r");
}


#endif
