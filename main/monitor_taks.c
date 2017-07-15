/*
 * monitor_taks.c
 *
 *	This Task waits for various events and controls other tasks
 *
 *  Created on: Jun 19, 2016
 *      Author: Visakhan C
 */

#include <string.h>
#include "stm32f1_rtc.h"
#include "debug_console.h"
#include "config_private.h"
#include "board.h"
#include "gsm_common.h"
#include "gps_common.h"
#include "stm32f1_timer.h"

extern void log_task_switch(void);
extern void SystemClock_Config(void);


static TaskHandle_t		xBatteryTaskHandle;
static TimerHandle_t	buttonTimer;
static volatile bool 	buttonTimerStarted;

static void ButtonTimerCallback(TimerHandle_t xTimer);

void monitor_task(void *pvParameters)
{
	EventBits_t ev;
	uint8_t gps_cmd[] = "$PMTK161,0*28\r\n";

	/* Initialize timer for measuring NETLIGHT pin signal */
	Timer_Init(TIM2, SystemCoreClock/1000, 4000);

	/* Create Timer for monitoring power button */
	buttonTimer = xTimerCreate("BUT_TMR", 200, pdTRUE, (void *)0, ButtonTimerCallback);

	while(1) {

		/* Wait for call to begin logging */
		ev = gsm_wait_for_event(EVENT_GSM_RING | EVENT_GSM_POWERSWITCH | EVENT_BUTTON_PRESS, 0);
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
						log_task_switch();
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
				if(0 != gsm_uart_acquire()) { DEBUG_PUTS("monitor: uart_acquire failed\r\n"); }
				gsm_send_command("AT+QPOWD=0");
				if(0 != gsm_uart_release()) { DEBUG_PUTS("monitor: uart_release failed\r\n"); }
				vTaskDelay(300);
				gsm_status.power_state = GSM_POWERDOWN;
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
		if(ev & EVENT_BUTTON_PRESS) {
			if(gsm_status.power_state == GSM_POWERON) {
				LED_On();
				DEBUG_PUTS("\r\n...Entering SYSTEM SLEEP...\r\n");
				if(0 != gsm_uart_acquire()) { DEBUG_PUTS("monitor: uart_acquire failed\r\n"); }
				/* Set GSM Module in sleep mode */
				gsm_send_command("AT+QSCLK=1");
				if(0 != gsm_uart_release()) { DEBUG_PUTS("monitor: uart_release failed\r\n"); }
				if(gsm_wait_for_event(EVENT_GSM_OK, 1000) & EVENT_GSM_OK) {
					HAL_GPIO_WritePin(GSM_DTR_GPIO_PORT, GSM_DTR_PIN, GPIO_PIN_SET);
				}
				else {
					DEBUG_PUTS("GSM Sleep error\r\n");
				}
				/* Set GPS Module in Standby mode */
				if(gps_send_cmd(gps_cmd, strlen(gps_cmd)) != 0) {
					DEBUG_PUTS("GPS Standby error\r\n");
				}
				LED_Off();
				/* MCU Power-down */
				DEBUG_PUTS("\r\nENTERING STOP...\r\n");
				RTC_Set_Counter(0);
				RTC_Alarm_Config(15);
				__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
				HAL_PWR_EnterSTOPMode(PWR_MAINREGULATOR_ON, PWR_STOPENTRY_WFI);
				/* Setup HSE clock configuration again */
				SystemClock_Config();
				DEBUG_PUTS("\r\n...BACK FROM STOP\r\n");
				/* Set GPS in Full on mode */
				gps_send("abc", 3);
				/* Put GSM back to normal mode */
				HAL_GPIO_WritePin(GSM_DTR_GPIO_PORT, GSM_DTR_PIN, GPIO_PIN_RESET);
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

/**
 * @brief Callback for FreeRTOS Timer to sample user button pressed state
 * @details Timer starts when user presses button. After each timer period(200ms),
 * this callback is called. If button is no longer pressed, the timer is stopped.
 * If button remains pressed the timer continues to run. If timer elapses 12 times,
 * indicating 2.4sec press duration, the Power switch event is set and timer is stopped
 *
 * @param xTimer Timer handle
 */
static void ButtonTimerCallback(TimerHandle_t xTimer)
{
	uint32_t count = (uint32_t) pvTimerGetTimerID(xTimer);
	GPIO_PinState	pin_state = HAL_GPIO_ReadPin(BUTTON_GPIO_PORT, BUTTON_PIN);
	count++;
	vTimerSetTimerID(xTimer, (void *)count);

	if(count == 1) {
		//gsm_set_event(EVENT_BUTTON_PRESS);  /* TESTING: Used to enter Low power mode */
	}

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

/**
 * @brief IRQ Handler for EXTI0 External interrupt
 * @note Used to handle on-board user button
 */
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

/**
 * @brief IRQ Handler for EXTI3 External interrupt
 * @note Used to handle battery charging indicator GPIO
 */
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

/**
 * @brief IRQ Handler for EXTI15_10 External interrupts
 * @note Used for handling NETLIGHT GPIO of GSM module
 */
void EXTI15_10_IRQHandler(void)
{
	GPIO_PinState netlight_pin = 0;
	uint32_t timer_val = TIM2->CNT;

	/* Clear interrupt flag on EXTI14 pin */
	if(EXTI->PR & EXTI_PR_PIF14) {
		EXTI->PR = EXTI_PR_PIF14;
		//netlight_pin = HAL_GPIO_ReadPin(GSM_NETLIGHT_GPIO_PORT, GSM_NETLIGHT_PIN);
		//(netlight_pin == GPIO_PIN_SET) ? LED_On() : LED_Off();

		if((timer_val > 1900) && (timer_val < 2100)) {
			/* 2000ms duration => GSM registered */
			gsm_status.registered = true;
		}
		else { /*((timer_val > 700) && (timer_val < 900)) */
			/* 800ms duration => GSM not registered */
			gsm_status.registered = false;
		}
		/* Reset timer */
		TIM2->CNT = 0;
	}
}


/**
 * @brief IRQ Handler for Timer2 timer
 * @note Timer 2 is used to measure period of NETLIGHT pin signal
 */
void TIM2_IRQHandler(void)
{
	TIM2->SR = 0;
	/* Timer overflow means 4 sec elapsed without NETLIGHT signal edge */
	gsm_status.registered = false;
}







