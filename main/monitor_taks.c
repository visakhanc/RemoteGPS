/*
 * monitor_taks.c
 *
 *	This Task wsits for commands from remote user and controls other tasks
 *
 *  Created on: Jun 19, 2016
 *      Author: Visakhan C
 */

#include "board.h"
#include "gsm_common.h"
#include "gps_common.h"
#include "debug_console.h"
#include "config_private.h"
#include <string.h>

static TaskHandle_t		xBatteryTaskHandle;
static TimerHandle_t	buttonTimer;
static volatile bool 	buttonTimerStarted;
//extern void log_task_switch(void);


static void ButtonTimerCallback(TimerHandle_t xTimer);

void monitor_task(void *pvParameters)
{
	EventBits_t ev;

	/* Create Timer for monitoring power button */
	buttonTimer = xTimerCreate("TIMER0", 200, pdTRUE, (void *)0, ButtonTimerCallback);
#if 0
	vTaskDelay(2000);
	if(0 == gps_send_cmd((uint8_t *)gps_cmd, strlen(gps_cmd))) {
		DEBUG_PUTS("GPS command OK\r\n");
	}
#endif
	while(1) {

		/* Wait for call to begin logging */
		ev = gsm_wait_for_event(EVENT_GSM_RING | EVENT_GSM_POWERSWITCH, 0);
		if(ev & EVENT_GSM_RING) {
			ev = gsm_wait_for_event(EVENT_GSM_CLIP, 1000);
			if (ev & EVENT_GSM_CLIP) {
				DEBUG_PUTS("\r\nCall from ");
				Debug_Console_PutBuf(gsm_status.caller, strlen((char *)gsm_status.caller));
				DEBUG_PUTS("\r\n");

				/* Get access to GSM UART */
				//if(gsm_uart_acquire() == 0) { // TODO: Cannot acquire uart during HTTP GET session. Caller has to disconnect himself

					/* Disconnect call */
					gsm_send_command("ATH");
					gsm_wait_for_event(EVENT_GSM_OK, 500);
					
					/* Process only if number is registered */
					if (0 == strcmp((char *)gsm_status.caller, AUTH_CALLER)) {
						/* Start/Stop logging */
						//log_task_switch();
					}

					/* Release uart access */
					//if(gsm_uart_release() != 0) {
					//	PRINTF("\r\nmonitor: uart release failed");
					//}

				//}
				//else {
				//	PRINTF("\r\nmonitor: gsm_uart_acquire() failed");
				//}
			}

		}
		if(ev & EVENT_GSM_POWERSWITCH) {
			if(gsm_status.power_state == GSM_POWERON) {
				/* Power off GSM Module */
				LED_On();
				gsm_send_command("AT+QPOWD=1");
				vTaskDelay(300);
				gsm_status.power_state = GSM_POWERDOWN;
				gsm_status.registerd = false;
				LED_Off();
				DEBUG_PUTS("POWEROFF...\r\n");
			}
			else if(gsm_status.power_state == GSM_POWERDOWN) {
				/* Power on GSM module */
				LED_On();
				HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_PORT, GSM_PWRKEY_PIN, GPIO_PIN_SET);
				vTaskDelay(1500);
				HAL_GPIO_WritePin(GSM_PWRKEY_GPIO_PORT, GSM_PWRKEY_PIN, GPIO_PIN_RESET);
				LED_Off();
				gsm_status.power_state = GSM_POWERON;
				DEBUG_PUTS("POWERED ON...\r\n");
			}
		}
	}
}

/*
 * 	Task to monitor battery charging
 */
void battery_task(void *pvParams)
{
	GPIO_PinState 	charge_pin;
	uint32_t 		ulNotificationValue;

	xBatteryTaskHandle = xTaskGetCurrentTaskHandle();
	charge_pin = HAL_GPIO_ReadPin(BAT_CHRG_GPIO_PORT, BAT_CHRG_PIN);
	board_state.charging = (charge_pin == GPIO_PIN_RESET) ? true : false;

	while(1)
	{
		/* Wait for notification from ISR */
		if((ulNotificationValue = ulTaskNotifyTake(pdTRUE, portMAX_DELAY)) == 1) {
			if(board_state.charging == true) {
				DEBUG_PUTS("CHARGING...\r\n");
				LED_On();
			}
			else {
				DEBUG_PUTS("NOT CHARGING...\r\n");
				LED_Off();
			}
		}
	}
}

static void ButtonTimerCallback(TimerHandle_t xTimer)
{
	uint32_t count = (uint32_t) pvTimerGetTimerID(xTimer);
	GPIO_PinState	pin_state = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN);
	count++;
	vTimerSetTimerID(xTimer, (void *)count);
	/* If Button is no longer pressed (less than 2.4 sec) or pressed for 2.4 sec exactly */
	if((pin_state == GPIO_PIN_RESET) || ((pin_state == GPIO_PIN_SET) && (count == 12)) ) {
		/* Stop timer and reset timer ID value */
		xTimerStop(buttonTimer, 0);
		vTimerSetTimerID(buttonTimer, (void *)0);
		buttonTimerStarted = false;
		/* If pressed exactly for 2.4 sec, issue POWEROFF event */
		if(count == 12) {
			gsm_set_event(EVENT_GSM_POWERSWITCH);
		}
	}
}

void EXTI0_IRQHandler(void)
{
	BaseType_t 	xHigherPriorityTaskWoken = pdFALSE;

	/* Clear EXTI0 pending bit */
	EXTI->PR = EXTI_PR_PIF0;

	/* Start the timer for monitoring button status */
	if(!buttonTimerStarted) {
		buttonTimerStarted = true;
		xTimerStartFromISR(buttonTimer, &xHigherPriorityTaskWoken);
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void EXTI3_IRQHandler(void)
{
	BaseType_t 	xHigherPriorityTaskWoken = pdFALSE;
	GPIO_PinState charge_pin;

	/* Clear EXTI3 pending bit */
	EXTI->PR = EXTI_PR_PIF3;

	/* Get current status of charging indicator pin */
	charge_pin = HAL_GPIO_ReadPin(BAT_CHRG_GPIO_PORT, BAT_CHRG_PIN);
	if(charge_pin == GPIO_PIN_SET) {
		board_state.charging = false;
	}
	else {
		board_state.charging = true;
	}
	/* Notify battery monitoring task */
	if(xBatteryTaskHandle) {
		vTaskNotifyGiveFromISR(xBatteryTaskHandle, &xHigherPriorityTaskWoken);
		portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
	}
}

void EXTI15_10_IRQHandler(void)
{
	GPIO_PinState netlight_pin = 0;

	/* Clear interrupt flag on EXTI14 pin */
	if(EXTI->PR & EXTI_PR_PIF14) {
		EXTI->PR = EXTI_PR_PIF14;
		netlight_pin = HAL_GPIO_ReadPin(GSM_NETLIGHT_GPIO_PORT, GSM_NETLIGHT_PIN);
		(netlight_pin == GPIO_PIN_SET) ? LED_On() : LED_Off();
	}
}
