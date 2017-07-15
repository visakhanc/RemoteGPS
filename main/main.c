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
#include "stm32f1_adc.h"
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
#define	LOG_TASK_STACKSIZE			256
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
	/* Initialize RTC clock (for first time)*/
	RTC_Init();
	/* Configure the System clock */
	SystemClock_Config();
#if USB_ENABLED
	/* Set USB prescaler = 1 */
	__HAL_RCC_USB_CONFIG(1 << 22);
#endif
#if ADC_ENABLED
	Adc_Init();
#endif

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
	//uint16_t 	adc_val;
	volatile uint32_t count = 0;
	uint8_t 	checksum;
	uint32_t 	len;

	LED_On();
	DEBUG_PUTS("STARTING UP....\r\n");
	snprintf(print_buf, sizeof(print_buf), "SysClock : %d\r\n", (int)SystemCoreClock);
	Debug_Console_PutBuf((char *)print_buf, strlen(print_buf));
	while(1)
	{
#if ADC_ENABLED
		for(len = 0; len < 7; len++) {
			adc_val = Adc_Sample_Polled(BAT_MON_ADC_CHANNEL);
			if(len == 4) {
				sprintf((char *)print_buf, "ADC Val: 0x%04x\r\n", adc_val);
				Debug_Console_PutBuf(print_buf, strlen((char *)print_buf));
			}
		}
#endif

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
			case NO_FIX: DEBUG_PUTS("No Fix"); break;
			case GPS_FIX: DEBUG_PUTS("GPS Fix"); break;
			case DGPS_FIX: DEBUG_PUTS("DGPS Fix"); break;
			default: DEBUG_PUTS("Invalid data");
		}
#endif
		snprintf(print_buf, sizeof(print_buf), "\r\nLat: %d.%d %c", (int)gps_info.latitude, (int)((gps_info.latitude * 1000.0) - (int)gps_info.latitude * 1000), gps_info.noth_south);
		Debug_Console_PutBuf((uint8_t *)print_buf, strlen(print_buf));
		snprintf(print_buf, sizeof(print_buf), "\tLon: %d.%d %c", (int)gps_info.longitude, (int)((gps_info.longitude * 1000.0) - (int)gps_info.longitude * 1000), gps_info.east_west);
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

		vTaskDelay(3000);
	}

}

/* Task to indicate GSM/GPS status through LED
 * --| GSM_STATUS <--500ms--> GPS_STATUS -------2000ms--------|--
 * 	Status blinks as follows:
 * 	2 blinks : STATUS NOT OK
 * 	1 blink  : STATUS OK
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
		if(NO_FIX == gps_info.fix) {
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
		//LED_Toggle();
	}
	if(RTC->CRL & RTC_CRL_ALRF) {
		RTC->CRL &= ~RTC_CRL_ALRF;
		//LED_Toggle();
		/* Clear RTC Counter for next Alarm */
		//RTC->CRL |= RTC_CRL_CNF;
		//RTC->CNTL = 0;
		//RTC->CRL &= ~RTC_CRL_CNF;
		/* Notify Log task */
		notify_next_log();
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
		//notify_next_log();
	}
}

/* RCC Interrupt handler - to know when LSE oscillator becomes ready */
void RCC_IRQHandler(void)
{
	if(RCC->CIR & RCC_CIR_LSERDYF) {
		RCC->CIR |= RCC_CIR_LSERDYC;
		//LED_On();
	}
}



/**
  * @brief  System Clock Configuration
  *         The system Clock is configured as follow :
  *            System Clock source            = PLL (HSI)
  *            SYSCLK(Hz)                     = 64000000
  *            HCLK(Hz)                       = 64000000
  *            AHB Prescaler                  = 1
  *            APB1 Prescaler                 = 2
  *            APB2 Prescaler                 = 1
  *            PLLMUL                         = 16
  *            Flash Latency(WS)              = 2
  * @param  None
  * @retval None
  */
void SystemClock_Config(void)
{
  /* Initialize HSE Oscillator */
  RCC_HSE_OscInit(false);

  /* Initialize PLL with HSE as source. Multiply by 6 => 8MHz x 6 = 48MHz */
  RCC_PLL_Init(RCC_PLLSOURCE_HSE, RCC_PLL_MUL3);

  /* Configure System clock to use PLL */
  RCC_Clock_Config(RCC_SYSCLKSOURCE_PLLCLK);

  /* Disable HSI clock */
  RCC_HSI_Disable();
}





