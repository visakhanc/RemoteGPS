/*
 * log_task.c
 *
 *	This task is responsible for sending location data to a web server
 *	at regular intervals
 *
 *  Created on: Jun 17, 2016
 *      Author: Visakhan C
 */

#include <gps_common.h>
#include "config_private.h"
#include "gsm_common.h"
#include "gps_common.h"
#include "http_utils.h"
#include "num_utils.h"
#include "debug_console.h"
#include "board.h"
#include "stm32f1_rtc.h"
#include "stm32f1_rcc.h"

/* Macros */
#define EVENT_START_LOGGING  	(1 << 0)
#define EVENT_STOP_LOGGING		(1 << 1)
#define EVENT_NEXT_LOG			(1 << 2)

#define LOG_URL_OFFSET (sizeof(LOG_API_URL)-1)

/* Globals */
extern volatile int gps_count;
extern volatile bool gps_progress;
extern uint8_t http_buf[];

static char logPostParams[128];
static TaskHandle_t xLogTaskHandle;
static char printBuf[80];
static volatile bool logEnabled = false;
static volatile uint32_t sleepInterval = DEFAULT_SLEEP_INTERVAL;
static volatile uint32_t logInterval;
static uint8_t gpsBackupCmd[] = "$PMTK225,4*2F\r\n";
static uint8_t gpsSetCmd[] = "$PMTK314,0,1,0,1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0*28\r\n";

extern void SystemClock_Config(void);


void log_task(void *pvParameters)
{
	int ret;
	EventBits_t ev;
	float curLat = 9.620093;
	float curLon = 76.618476;
	float prevLat = 0.0;
	float prevLon = 0.0;
	int offset;
	uint32_t notifValue = 0;
	volatile int count = 0;
	bool retry = false;
	char *bufPtr;
	int rtcVal = 0;

	xLogTaskHandle = xTaskGetCurrentTaskHandle();
	do {
		/* Wait until GSM module get registered */
		vTaskDelay(1000);
	} while (gsm_status.registered != true);
	DEBUG_PUTS("Log Task...\r\n");
	/* Activate PDP Context */
	gsm_clear_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME);
	gsm_uart_acquire();
	gsm_send_command("AT+QIREGAPP");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME, 1000);
	if(ev & EVENT_GSM_OK) {
		gsm_send_command("AT+QIACT");
	}
	else {
		Debug_Console_PutBuf(printBuf, sprintf(printBuf, "IREGAPP err: 0x%08lx\r\n", ev));
	}
	/* While activating PDP context, wait for GPS Fix... */
	if(true == gps_info.ext_antenna) {
		DEBUG_PUTS("EXT ANT Detected: 8 sec ALARM\r\n");
		logInterval = DEFAULT_LOG_INTERVAL;
	}
	else {
		DEBUG_PUTS("No EXT ANT: 30 sec ALARM\r\n");
		logInterval = 30;
	}
	if(gps_send_cmd(gpsSetCmd, sizeof(gpsSetCmd)-1) != 0) {
		DEBUG_PUTS("GPS Set error...\r\n");
	}
	/* Wait for GPS Fix */
	DEBUG_PUTS("Waiting GPS Fix...");
	while(gps_info.fix == GPS_NOFIX) {
		vTaskDelay(500);
	}
	DEBUG_PUTS("OK\r\n");

	/* Wait for PDP context(AT+QIACT response) */
	if(ev & EVENT_GSM_OK) {
		ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME, 40000);
		if(!(ev & EVENT_GSM_OK)) {
			Debug_Console_PutBuf(printBuf, sprintf(printBuf, "IACT err: 0x%08lx\r\n", ev));
		}
		else {
			DEBUG_PUTS("Internet enabled\r\n");
		}
	}
	gsm_uart_release();

	/* Trigger the first log after 2 sec from now */
	RTC_Alarm_Config(RTC_Get_Count() + 2);

	while(1)
	{
		/* Wait for Notification to start/stop logging */
		if(pdTRUE == xTaskNotifyWait(0, -1UL, &notifValue, portMAX_DELAY)) {
			if(notifValue & EVENT_NEXT_LOG) {
				Debug_Console_PutBuf(printBuf, sprintf(printBuf, "\r\nNext Log...(%d)\r\n", count));
			}
			else if(notifValue & EVENT_START_LOGGING) {
				DEBUG_PUTS("\r\nLogging enabled");
				RTC_Alarm_Config(RTC_Get_Count() + logInterval - 1);
				continue;
			}
			else if(notifValue & EVENT_STOP_LOGGING) {
				// TODO: disable logging on server too?
				DEBUG_PUTS("\r\nLogging disabled");
				RTC_Alarm_Config(RTC_Get_Count() + logInterval - 1);
				continue;
			}
		}

		if((!count) || (true == logEnabled)) {
#if LOG_ENABLED
			/* Log only if position is reasonably accurate */
			//if((gps_info.fix > GPS_NOFIX) && (gps_info.hdop < 2.0)) {

				curLat = gps_info.latitude + GPS_OFFSET;
				curLon = gps_info.longitude + GPS_OFFSET;
				if((curLat != prevLat) || (curLon != prevLon)) {
					gps_progress = true;
				}
				offset = 0;
				offset += sprintf(&logPostParams[offset], "lat=");
				offset += float_to_string(&curLat, 6, (uint8_t *)&logPostParams[offset]);
				offset += sprintf(&logPostParams[offset], "&lon=");
				offset += float_to_string(&curLon, 6, (uint8_t *)&logPostParams[offset]);
				offset += sprintf(&logPostParams[offset], "&sp=%d&dr=%d&hd=%d", gps_info.velocity, gps_info.course, (int)(gps_info.hdop*10.0));
				offset += sprintf(&logPostParams[offset], "&dt=%d-%02d-%02d&tm=%02d:%02d:%02d",
								  gps_info.time.year+2000, gps_info.time.month, gps_info.time.date,
								  gps_info.time.hour, gps_info.time.min, gps_info.time.sec);
				offset += sprintf(&logPostParams[offset], "&cnt=%d&pr=%s&id=%d", gps_count,
								  	  	  (gps_progress == true) ? "true" : "false", count);
				offset += sprintf(&logPostParams[offset], "&ig=%s", (Battery_Charging() ? "on" : "off"));

				DEBUG_PUTS("Logging...\r\n");
				/* acquire gsm uart */
				gsm_uart_acquire();
				ev = gsm_wait_for_event(EVENT_GSM_CLIP, 1);
				Debug_Console_PutBuf(printBuf, sprintf(printBuf, "PRE EVENTS: 0x%08lx\r\n", ev));
				gsm_clear_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME|EVENT_GSM_DEACT);
#if 1
				/* Verify GPRS status is OK */
				gsm_send_command("AT+QISTAT");
				ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME, 1000);
				if(ev & EVENT_GSM_OK) {
					vTaskDelay(50);
					Debug_Console_PutBuf(printBuf, sprintf(printBuf, "gprs_state: %d\r\n", gsm_status.gprs_state));
					if(gsm_status.gprs_state != GPRS_ACT) {
						if(gsm_status.gprs_state == GPRS_DEACT) {
							gsm_send_command("AT+QIDEACT");
							ev = gsm_wait_for_event(EVENT_GSM_DEACT|EVENT_GSM_ERROR|EVENT_GSM_CME, 5000);
							if(ev & EVENT_GSM_DEACT) {
								DEBUG_PUTS("Deact ok\r\n");
								gsm_send_command("AT+QISTAT");
								gsm_wait_for_event(EVENT_GSM_OK, 500);
								vTaskDelay(500);
							}
							else {
								Debug_Console_PutBuf(printBuf, sprintf(printBuf, "Deact err: 0x%08lx", ev));
							}
						}
						if(gsm_status.gprs_state == GPRS_INITIAL) {
							gsm_send_command("AT+QIREGAPP");
							ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME, 1000);
							if(ev & EVENT_GSM_OK) {
								gsm_send_command("AT+QIACT");
								ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 30000);
								if(!(ev & EVENT_GSM_OK)) {
									Debug_Console_PutBuf(printBuf, sprintf(printBuf, "Re IACT err (0x%08lx) (cme:%d)\r\n", ev, gsm_status.cme_error));
								}
							}
							else {
								Debug_Console_PutBuf(printBuf, sprintf(printBuf, "Re IREG err (0x%08lx) (cme:%d)\r\n", ev, gsm_status.cme_error));
							}
						}
					}
				}
				else {
					Debug_Console_PutBuf(printBuf, sprintf(printBuf, "QISTAT ev: 0x%08lx  CME:%d\r\n", ev, gsm_status.cme_error));
				}
#endif
				gsm_clear_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME|EVENT_GSM_DEACT);
				gsm_send_command("AT+CBC");
				if(gsm_wait_for_event(EVENT_GSM_OK, 1000) & EVENT_GSM_OK) {
					offset += sprintf(&logPostParams[offset], "&bat=%d", gsm_status.bat_voltage);
				}
				/* Send location to server using HTTP POST method */
				ret = http_post(LOG_API_URL, sizeof(LOG_API_URL)-1, (uint8_t *)logPostParams, offset);
				/* Release gsm uart */
				gsm_uart_release();
				if(HTTP_OK == ret) {
					if(strstr((char *)http_buf, "OK") != NULL) {
						snprintf(printBuf, sizeof(printBuf), "POST success (response: %d B )\r\n", gsm_status.http_recv_len);
						Debug_Console_PutBuf(printBuf, strlen(printBuf));
						Debug_Console_PutBuf((char *)http_buf, gsm_status.http_recv_len);
						DEBUG_PUTS("\r\n");
						count++;
						retry = false;
						if((bufPtr = strstr((char *)http_buf, "log:")) != NULL) {
							bufPtr += sizeof("log:") - 1;
							logEnabled = (bufPtr[0] == '1');
						}
						bufPtr = strstr((char *)http_buf, "logper:");
						if(bufPtr) {
							bufPtr += sizeof("logper:") - 1;
							parse_decimal(&ret, bufPtr, 3);
							if(ret >= MINIMUM_LOG_INTERVAL) {
								logInterval = (uint32_t)ret;
							}
						}
						bufPtr = strstr((char *)http_buf, "sleep:");
						if(bufPtr) {
							bufPtr += sizeof("sleep:") - 1;
							parse_decimal(&ret, bufPtr, 5);
							if(ret >= MINIMUM_SLEEP_INTERVAL) {
								sleepInterval = (uint32_t)ret;
							}
						}
						Debug_Console_PutBuf(printBuf, sprintf(printBuf, "Log:%d, Logper:%lu, Sleep:%lu\r\n", logEnabled, logInterval, sleepInterval));
						if(logEnabled) {
							DEBUG_PUTS("Continuing Log..\r\n");
						}
						else {
							DEBUG_PUTS("Stopping Log.\r\n");
						}
					}
					else {
						snprintf(printBuf, sizeof(printBuf), "POST: Error from server (response: %d B )\r\n", gsm_status.http_recv_len);
						Debug_Console_PutBuf(printBuf, strlen(printBuf));
						Debug_Console_PutBuf((char *)http_buf, gsm_status.http_recv_len);
						logEnabled = false;
						DEBUG_PUTS("\r\nStopping Log..\r\n");
						count++;  // To not log next time;
					}
					if(logInterval >= 50) {
						vTaskDelay(1000);
						gsm_send_command("AT+QIDEACT");
						ev = gsm_wait_for_event(EVENT_GSM_DEACT, 10000);
						if(!(ev & EVENT_GSM_DEACT)) {
							Debug_Console_PutBuf(printBuf, sprintf(printBuf, "Deact failed. ev: 0x%08lx\r\n", ev));
						}
					}
				}
				else {
					sprintf(printBuf, "log: POST Err: 0x%08x (cme: %d) (url: %d B)\r\n", ret, gsm_status.cme_error, offset);
					Debug_Console_PutBuf(printBuf, strlen(printBuf));
					Debug_Console_PutBuf(logPostParams, strlen(logPostParams));
					vTaskDelay(1000);
					DEBUG_PUTS("\r\nDeact GPRS...");
					gsm_uart_acquire();
					gsm_clear_event(EVENT_GSM_DEACT);
					gsm_send_command("AT+QIDEACT");
					ev = gsm_wait_for_event(EVENT_GSM_DEACT, 20000);
					gsm_uart_release();
					if(ev & EVENT_GSM_DEACT) {
						DEBUG_PUTS("OK\r\n");
					}
					else {
						Debug_Console_PutBuf(printBuf, sprintf(printBuf, "Timeout ev: 0x%08lx; cme: %d\r\n", ev, gsm_status.cme_error));
					}
					/* Failed to log first time? - retry now */
					if((retry == false) && ((0 == count) || (logInterval >= 50))) {
						RTC_Alarm_Config(RTC_Get_Count() + 2);
						retry = true;
						DEBUG_PUTS("Retrying..\r\n");
						continue;
					}
				}

				prevLat = curLat;
				prevLon = curLon;
				gps_count = 0;
				gps_progress = false;

			//}
#endif  /* LOG_ENABLED */
		}
		/* When ignition is off, Go to SLEEP interval mode */
		if(Battery_Charging() == false) {
			DEBUG_PUTS("\r\n...Entering SYSTEM SLEEP...");
			/* Set GPS Module in Standby mode */
			GPS_ForceOn_Low();
			if(gps_send_cmd(gpsBackupCmd, sizeof(gpsBackupCmd)-1) != 0) {
				DEBUG_PUTS("GPS backup error\r\n");
			}
			/* Turn-off GSM Module */
			gsm_poweroff();
			/* Calculate offset required for Alarm value */
			if(rtcVal) {
				rtcVal = (int)RTC_Get_Count() - rtcVal;
			}
			/* Set ALARM for Sleep interval */
			RTC_Alarm_Config(RTC_Get_Count() + sleepInterval - rtcVal);
			LED_Off();
			/* Disable NETLIGHT interrupt and clear any pending interrupt flag */
			EXTI->IMR &= ~EXTI_IMR_IM14;
			EXTI->PR = EXTI_PR_PIF14;
			__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
			/* Enter MCU Stop mode */
			if(Battery_Charging() == false) { /* Enter MCU STOP mode if ignition still off */
				GPIO_Set_LowPower();
				HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
				/* Back from STOP mode...Setup HSE clock configuration again */
				SystemClock_Config();
				GPIO_Set_Normal();
				DEBUG_PUTS("\r\n...BACK FROM STOP\r\n");
				rtcVal = (int)RTC_Get_Count();
			}
			/* Set GPS in Full on mode (if already in backup mode) */
			GPS_ForceOn_High();
			/* Enable NETLIGHT interrupt  */
			EXTI->IMR |= EXTI_IMR_IM14;
			/* Turn-on GSM module */
			gsm_start();
			count = 0; /* Start a new set of log */
			/* If woken up by CHARGING interrupt (not ALARM), generate ALARM to start logging NOW */
			RTC_Alarm_Config(RTC_Get_Count() + 2);
			/* Activate PDP Context */
			gsm_clear_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME);
			gsm_uart_acquire();
			gsm_send_command("AT+QIREGAPP");
			ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME, 1000);
			if(ev & EVENT_GSM_OK) {
				gsm_send_command("AT+QIACT");
			}
			else {
				Debug_Console_PutBuf(printBuf, sprintf(printBuf, "IREGAPP err: 0x%08lx\r\n", ev));
			}
			while(gps_info.fix == GPS_NOFIX) {
				vTaskDelay(500);
			}
			if(ev & EVENT_GSM_OK) {
				ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME, 40000);
				if(!(ev & EVENT_GSM_OK)) {
					Debug_Console_PutBuf(printBuf, sprintf(printBuf, "IACT err: 0x%08lx\r\n", ev));
				}
			}
			gsm_uart_release();
		}
		else {
			DEBUG_PUTS("Set ALARM...\r\n");
			/* Setup Alarm for next iteration of log */
			RTC_Alarm_Config(RTC_Get_Count() + logInterval - 1);
		}

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


/* To be called externally inside RTC ISR to log next sample
 * (This is another way for periodic logging, instead of Notify wait timeout) */
void notify_next_log(void)
{
	BaseType_t 	xHigherPriorityTaskWoken = pdFALSE;

	if(NULL == xLogTaskHandle) {
		return;
	}
	xTaskNotifyFromISR(xLogTaskHandle, EVENT_NEXT_LOG, eSetBits, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}



