/*
 * main.c
 *
 *	Contains main function which initializes all the application tasks and the
 *	FreeRTOS scheduler
 *
 *  Created on: Apr 5, 2017
 *      Author: Visakhan C
 */

/* Includes ------------------------------------------------------------------*/
#include <stdbool.h>
#include <string.h>

#include "stm32f1xx_hal.h"
#include "stm32f1_uart.h"
#include "stm32f1_rtc.h"
#include "stm32f1_rcc.h"
#include "stm32f1xx_hal_pwr.h"
#include "stm32f1_timer.h"

#include "board.h"
#include "debug_console.h"
#include "gps_common.h"
#include "gsm_common.h"
#include "num_utils.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"


extern volatile int gps_count;
extern PCD_HandleTypeDef hpcd;

void Timer_Init(TIM_TypeDef *timer, uint32_t prescaler, uint32_t reload);
extern void gps_task(void *params);
extern void gsm_rx_task(void *params);
#if GSM_DEBUG == 1
extern void gsm_debug_task(void *params);
#endif
extern void message_task(void *params);
extern void monitor_task(void *params);
extern void battery_task(void *params);
extern void log_task(void *pvParameters);
extern void notify_next_log(void);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define GPS_TASK_PRIO			(configMAX_PRIORITIES - 1)
#define GSM_RX_TASK_PRIO		(configMAX_PRIORITIES - 1)
#define MESSAGE_TASK_PRIORITY	(configMAX_PRIORITIES - 2)
#define MONITOR_TASK_PRIORITY	(configMAX_PRIORITIES - 2)
#define LOG_TASK_PRIORITY		(configMAX_PRIORITIES - 3)
#define USB_TASK_PRIORITY		(configMAX_PRIORITIES - 3)
#define GSM_DEBUG_TASK_PRIO		(configMAX_PRIORITIES - 4)
#define DISPLAY_TASK_PRIORITY	(configMAX_PRIORITIES - 5)
#define LED_TASK_PRIORITY		(configMAX_PRIORITIES - 5)
#define BATTERY_TASK_PRIORITY	(configMAX_PRIORITIES - 5)

#define GPS_TASK_STACKSIZE			256
#define GSM_RX_TASK_STACKSIZE		256
#define GSM_DEBUG_TASK_STACKSIZE	256
#define MESSAGE_TASK_STACKSIZE		256
#define MONITOR_TASK_STACKSIZE		256
#define	LOG_TASK_STACKSIZE			300
#define BATTERY_TASK_STACKSIZE		configMINIMAL_STACK_SIZE
#define DISPLAY_TASK_STACKSIZE		256
#define LED_TASK_STACKSIZE			configMINIMAL_STACK_SIZE

/* Private macro -------------------------------------------------------------*/
#define	CURSOR_STEP			0
/* Private variables ---------------------------------------------------------*/
#if USB_ENABLED
USBD_HandleTypeDef USBD_Device;
uint8_t HID_Buffer[4];
#endif
static char print_buf[80];
volatile static bool wakeup_flag = false;
/* Private function prototypes -----------------------------------------------*/
#if USB_ENABLED
static void GetPointerData(uint8_t *pbuf);
#endif

/* Private functions ---------------------------------------------------------*/
void SystemClock_Config(void);
#if USB_ENABLED
void HID_Mouse_Task(void *pArg);
#endif
void display_task(void *pArg);
void led_task(void *pArg);


/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{
	/* Initialize GPIO ports and external interrupts */
	Board_Init();
	/* Initialize RTC clock with LSI clock */
	RTC_Init_LSI();
	/* Configure the System clock */
	SystemClock_Config();
#if USB_ENABLED
	/* Set USB prescaler = 1 */
	__HAL_RCC_USB_CONFIG(1 << 22);
#endif
	if(PWR->CSR & PWR_CSR_SBF) {
		wakeup_flag = true;
	}
	/* Enable Wakeup from standby mode by WKUP pin rising edge */
	PWR->CSR |= PWR_CSR_EWUP;

	Debug_Console_Init();

	xTaskCreate(gps_task, "GPS", GPS_TASK_STACKSIZE, NULL, GPS_TASK_PRIO, NULL);
	xTaskCreate(gsm_rx_task, "GSM_RX", GSM_RX_TASK_STACKSIZE, NULL, GSM_RX_TASK_PRIO, NULL);
#if GSM_DEBUG == 1
	xTaskCreate(gsm_debug_task, "GSM_DEBUG", GSM_DEBUG_TASK_STACKSIZE, NULL, GSM_DEBUG_TASK_PRIO, NULL);
#else
	xTaskCreate(log_task, "LOG", LOG_TASK_STACKSIZE, NULL, LOG_TASK_PRIORITY, NULL);
	xTaskCreate(message_task, "MESSAGE", MESSAGE_TASK_STACKSIZE, NULL, MESSAGE_TASK_PRIORITY, NULL);
#endif
	xTaskCreate(monitor_task, "MONITOR", MONITOR_TASK_STACKSIZE, NULL, MONITOR_TASK_PRIORITY, NULL);
	xTaskCreate(display_task, "DISPLAY",	DISPLAY_TASK_STACKSIZE, NULL,	DISPLAY_TASK_PRIORITY, NULL);
	xTaskCreate(battery_task, "BATTERY", BATTERY_TASK_STACKSIZE, NULL, BATTERY_TASK_PRIORITY, NULL);
	xTaskCreate(led_task, "LED", LED_TASK_STACKSIZE, NULL, LED_TASK_PRIORITY, NULL);
#if USB_ENABLED
	xTaskCreate(HID_Mouse_Task, "USB",	256, NULL,	USB_TASK_PRIORITY, NULL);
#endif

	/* To ensure priority grouping is set to be as expected by FreeRTOS */
	NVIC_SetPriorityGrouping(0);
	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler */
	for (;;);

}


void display_task(void *pArg)
{
	//uint32_t count;
	//uint8_t gps_standby_cmd[] = "$PMTK161,0*28\r\n";
	//uint8_t gps_backup_cmd[] = "$PMTK225,4*2F\r\n";

	if(wakeup_flag == true) {
		DEBUG_PUTS("Back from STANDBY...\r\n");
	}
	DEBUG_PUTS("STARTING UP....\r\n");
	snprintf(print_buf, sizeof(print_buf), "SysClock : %d\r\n", (int)SystemCoreClock);
	Debug_Console_PutBuf((char*)print_buf, strlen(print_buf));

#if 0
	DEBUG_PUTS("GPS Fix...\r\n");
	while(gps_info.fix == GPS_NOFIX) {
		vTaskDelay(100);
	}

	while(gsm_status.registered != true) {
		vTaskDelay(100);
	}
	DEBUG_PUTS("Registered\r\n");
	gsm_send_command("AT+QSCLK=1");
	if(!(gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME, 1000) & EVENT_GSM_OK)) {
		DEBUG_PUTS("QSCLK err\r\n");
	}
#endif
	while(1)
	{

#if DEBUG_GPS
		DEBUG_PUTS("\r\n");
		snprintf(print_buf, sizeof(print_buf), "Time: %02d:%02d:%02d  %d/%d/%d",
				gps_info.time.hour, gps_info.time.min, gps_info.time.sec,
				gps_info.time.date, gps_info.time.month, gps_info.time.year+2000);
		Debug_Console_PutBuf((uint8_t*)print_buf, strlen(print_buf));
#if 0
		DEBUG_PUTS("\r\nFix : ");
		switch(gps_info.fix)
		{
			case GPS_NOFIX: DEBUG_PUTS("No Fix"); break;
			case GPS_FIX: DEBUG_PUTS("GPS Fix"); break;
			case DGPS_FIX: DEBUG_PUTS("DGPS Fix"); break;
			default: DEBUG_PUTS("Invalid data");
		}
#endif
		snprintf(print_buf, sizeof(print_buf), "\r\nLat: %d.%d", (int)gps_info.latitude, (int)((gps_info.latitude * 1000.0) - (int)gps_info.latitude * 1000));
		Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
		snprintf(print_buf, sizeof(print_buf), "\tLon: %d.%d", (int)gps_info.longitude, (int)((gps_info.longitude * 1000.0) - (int)gps_info.longitude * 1000));
		Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
		//snprintf(print_buf, sizeof(print_buf), "\n\rAltitude: %d m", (int)gps_info.altitude);
		//Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
		snprintf(print_buf, sizeof(print_buf), "\r\nSpeed:    %d kph", (int)gps_info.velocity);
		Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
		snprintf(print_buf, sizeof(print_buf), "\tCourse:   %d deg", (int)gps_info.course);
		Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
		snprintf(print_buf, sizeof(print_buf), "\r\nHDOP:     %d.%d", (int)gps_info.hdop, (int)((gps_info.hdop * 10.0) - (int)gps_info.hdop * 10));
		Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
		snprintf(print_buf, sizeof(print_buf), "\tSatellites used: %d\r\n", gps_info.sat_used);
		Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
		//snprintf(print_buf, sizeof(print_buf), "gps_count: %d, rmc: %d, gga: %d\r\n", gps_count, gps_rmc_count, gps_gga_count);
		//Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
#endif

#if 0
		//vTaskDelay(10000);
		while((gps_info.fix == GPS_NOFIX) || (gps_info.hdop > 1.0))
		{
			snprintf(print_buf, sizeof(print_buf), "Lat: %d.%d", (int)gps_info.latitude, (int)((gps_info.latitude * 1000.0) - (int)gps_info.latitude * 1000));
			Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
			snprintf(print_buf, sizeof(print_buf), "\tLon: %d.%d", (int)gps_info.longitude, (int)((gps_info.longitude * 1000.0) - (int)gps_info.longitude * 1000));
			Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
			snprintf(print_buf, sizeof(print_buf), "\tHDOP: %d.%d", (int)gps_info.hdop, (int)((gps_info.hdop * 10.0) - (int)gps_info.hdop * 10));
			Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
			snprintf(print_buf, sizeof(print_buf), "\tSatellites used: %d\r\n", gps_info.sat_used);
			Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
			vTaskDelay(1000);
		}
		DEBUG_PUTS("GPS Standby...");
		if(gps_send_cmd(gps_standby_cmd, strlen(gps_standby_cmd)) != 0) {
			DEBUG_PUTS("GPS Standby error\r\n");
		}
		DEBUG_PUTS("OK\r\n");
		vTaskDelay(10000);
		/* Set GPS in Full on mode */
		gps_send("abc", 3);
		DEBUG_PUTS("GPS Normal\r\n");
		vTaskDelay(2000);
		while((gps_info.fix == GPS_NOFIX) || (gps_info.hdop > 1.0))
		{
			snprintf(print_buf, sizeof(print_buf), "Lat: %d.%d", (int)gps_info.latitude, (int)((gps_info.latitude * 1000.0) - (int)gps_info.latitude * 1000));
			Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
			snprintf(print_buf, sizeof(print_buf), "\tLon: %d.%d", (int)gps_info.longitude, (int)((gps_info.longitude * 1000.0) - (int)gps_info.longitude * 1000));
			Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
			snprintf(print_buf, sizeof(print_buf), "\tHDOP: %d.%d", (int)gps_info.hdop, (int)((gps_info.hdop * 10.0) - (int)gps_info.hdop * 10));
			Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
			snprintf(print_buf, sizeof(print_buf), "\tSatellites used: %d\r\n", gps_info.sat_used);
			Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
			vTaskDelay(1000);
		}
#endif

#if 0 // DISABLE SLEEP TEST
		GPS_ForceOn_Low();
		DEBUG_PUTS("GPS backup...");
		if(gps_send_cmd(gps_backup_cmd, strlen(gps_backup_cmd)) != 0) {
			DEBUG_PUTS("GPS Backup error\r\n");
		}
		DEBUG_PUTS("OK\r\n");
		vTaskDelay(4000);
#if 0
		DEBUG_PUTS("GSM CFUN=0\r\n");
		gsm_send_command("AT+CFUN=0");
		if(!(gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME, 10000) & EVENT_GSM_OK)) {
			DEBUG_PUTS("CFUN err\r\n");
		}
		DEBUG_PUTS("GSM Sleep...\r\n");
		/* Set GSM Module in sleep mode */
		HAL_GPIO_WritePin(GSM_DTR_GPIO_PORT, GSM_DTR_PIN, GPIO_PIN_SET);
		vTaskDelay(5000);
#endif
		DEBUG_PUTS("Enter STOP..");
		/* Disable NETLIGHT interrupt and clear any pending interrupt flag */
		EXTI->IMR &= ~EXTI_IMR_IM14;
		EXTI->PR = EXTI_PR_PIF14;
		LED_Off();
		GPIO_Set_LowPower();
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
		__HAL_PWR_CLEAR_FLAG(PWR_FLAG_SB);
		/* MCU sleep mode */
		HAL_PWR_EnterSTOPMode(PWR_LOWPOWERREGULATOR_ON, PWR_STOPENTRY_WFI);
		//HAL_PWR_EnterSTANDBYMode();
		/* Setup HSE clock configuration again */
		SystemClock_Config();
		GPIO_Set_Normal();
		DEBUG_PUTS("\r\n...BACK FROM STOP\r\n");
		/* Enable NETLIGHT interrupt  */
		EXTI->IMR |= EXTI_IMR_IM14;
		vTaskDelay(4000);
#if 0
		DEBUG_PUTS("GSM Normal..");
		/* Put GSM back to normal mode */
		HAL_GPIO_WritePin(GSM_DTR_GPIO_PORT, GSM_DTR_PIN, GPIO_PIN_RESET);
#endif
		/* Set GPS in Full on mode */
		GPS_ForceOn_High();
		DEBUG_PUTS("GPS normal..\r\n");
#endif // DISABLE SLEEP TEST

		vTaskDelay(4000);
	}

}

/* Task to indicate GSM/GPS status through LED
 * --| GSM_STATUS <--500ms--> GPS_STATUS -------2000ms--------|--
 * 	Status blinks as follows:
 * 	2 blinks : STATUS NOT OK (GSM: Not registered; GPS: No fix)
 * 	1 blink  : STATUS OK (GSM: Registered; GPS: Fix)
 * 	3 blinks(GSM) : SIM card not present
 */
void led_task(void *pArg)
{

	while(1) {
		/* GSM Status */
		if(gsm_status.power_state == GSM_POWERDOWN) {
			LED_On();
			vTaskDelay(280);
			LED_Off();
		}
		else { /* GSM_POWERON */
			LED_On();
			vTaskDelay(40);
			LED_Off();
			vTaskDelay(200);
			if(false == gsm_status.sim_present) {
				LED_On();
				vTaskDelay(40);
				LED_Off();
				vTaskDelay(200);
				LED_On();
				vTaskDelay(40);
				LED_Off();
			}
			else if(false == gsm_status.registered) {
				LED_On();
				vTaskDelay(40);
				LED_Off();
			}
		}
		/* Delay */
		vTaskDelay(500);
		/* GPS status */
		LED_On();
		vTaskDelay(40);
		LED_Off();
		vTaskDelay(200);
		if(GPS_NOFIX == gps_info.fix) {
			LED_On();
			vTaskDelay(40);
			LED_Off();
		}
		vTaskDelay(2000);
	}
}

#if USB_ENABLED

void HID_Mouse_Task(void *pArg)
{
	/* Init Device Library */
	USBD_Init(&USBD_Device, &HID_Desc, 0);

	/* Register the HID class */
	USBD_RegisterClass(&USBD_Device, USBD_HID_CLASS);

	/* Start Device Process */
	USBD_Start(&USBD_Device);

	DEBUG_PUTS("USB Initialized\r\n");
	while(1)
	{
		vTaskDelay(200);
	    GetPointerData(HID_Buffer);
	    USBD_HID_SendReport(&USBD_Device, HID_Buffer, 4);
	}

}


/**
  * @brief  Gets Pointer Data.
  * @param  pbuf: Pointer to report
  * @retval None
  */
static void GetPointerData(uint8_t *pbuf)
{
  static int8_t cnt = 0;
  int8_t  x = 0, y = 0 ;

  if(cnt++ > 0)
  {
    x = CURSOR_STEP;
  }
  else
  {
    x = -CURSOR_STEP;
  }

  pbuf[0] = 0;
  pbuf[1] = x;
  pbuf[2] = y;
  pbuf[3] = 0;
}



/**
  * @brief  This function handles USB Handler.
  * @param  None
  * @retval None
  */
void USB_LP_CAN1_RX0_IRQHandler(void)
{
  HAL_PCD_IRQHandler(&hpcd);
}

/**
  * @brief  This function handles USB WakeUp interrupt request.
  * @param  None
  * @retval None
  */
void USBWakeUp_IRQHandler(void)
{
  __HAL_USB_WAKEUP_EXTI_CLEAR_FLAG();
}

#endif


void RTC_IRQHandler(void)
{
	if(RTC->CRL & RTC_CRL_SECF) {
		RTC->CRL &= ~RTC_CRL_SECF;
	}
	if(RTC->CRL & RTC_CRL_ALRF) {
		RTC->CRL &= ~RTC_CRL_ALRF;
	}
}

void RTC_Alarm_IRQHandler(void)
{
	/* Clear EXTI Line 17 (RTC)*/
	EXTI->PR = EXTI_PR_PIF17;
	/* Clear Wakeup flag, if RTC alarm caused a wakeup from STOP mode */
	__HAL_PWR_CLEAR_FLAG(PWR_FLAG_WU);
	if(RTC->CRL & RTC_CRL_ALRF) {
		RTC->CRL &= ~RTC_CRL_ALRF;
		/* Notify Log task */
		notify_next_log();
	}
}

/* RCC Interrupt handler - to know when LSE oscillator becomes ready */
void RCC_IRQHandler(void)
{
	if(RCC->CIR & RCC_CIR_LSERDYF) {
		RCC->CIR |= RCC_CIR_LSERDYC;
	}
}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 48000000
  *            HCLK(Hz)                       = 48000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Initialize HSE Oscillator */
  RCC_HSE_OscInit(false);

  /* Initialize PLL with HSE as source. Multiply by 6 => 8MHz x 6 = 48MHz */
  RCC_PLL_Init(RCC_PLLSOURCE_HSE, RCC_PLL_MUL6);

  /* Configure System clock to use PLL */
  RCC_Clock_Config(RCC_SYSCLKSOURCE_PLLCLK);

  /* Disable HSI clock */
  RCC_HSI_Disable();
}





