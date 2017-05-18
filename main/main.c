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
#include <gps_common.h>
#include <stdbool.h>
#include <string.h>
#include "stm32f1xx_hal.h"
#include "FreeRTOS.h"
#include "task.h"
#include "board.h"
#include "stm32f1_uart.h"
#include "stm32f1_adc.h"
#include "debug_console.h"
#include "gsm_common.h"
#include "usbd_desc.h"
#include "usbd_hid.h"

#define	DEBUG_GPS	1
extern gps_info_struct gps_info;
extern volatile int gps_count;
extern PCD_HandleTypeDef hpcd;

extern void gps_task(void *params);
extern void gsm_rx_task(void *params);
#if GSM_DEBUG == 1
extern void gsm_debug_task(void *params);
#endif
extern void message_task(void *params);
extern void monitor_task(void *params);
extern void battery_task(void *params);
extern void log_task(void *pvParameters);

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
#define GPS_TASK_PRIO			(configMAX_PRIORITIES - 1)
#define GSM_RX_TASK_PRIO		(configMAX_PRIORITIES - 1)
#define GSM_DEBUG_TASK_PRIO		(configMAX_PRIORITIES - 2)
#define MESSAGE_TASK_PRIORITY	(configMAX_PRIORITIES - 2)
#define MONITOR_TASK_PRIORITY	(configMAX_PRIORITIES - 2)
#define LOG_TASK_PRIORITY		(configMAX_PRIORITIES - 2)
#define BATTERY_TASK_PRIORITY	(configMAX_PRIORITIES - 3)
#define USB_TASK_PRIORITY		(configMAX_PRIORITIES - 2)
#define DISPLAY_TASK_PRIORITY	(configMAX_PRIORITIES - 3)

#define GPS_TASK_STACKSIZE			256
#define GSM_RX_TASK_STACKSIZE		256
#define GSM_DEBUG_TASK_STACKSIZE	256
#define MESSAGE_TASK_STACKSIZE		256
#define MONITOR_TASK_STACKSIZE		256
#define	LOG_TASK_STACKSIZE			256
#define BATTERY_TASK_STACKSIZE		configMINIMAL_STACK_SIZE
#define DISPLAY_TASK_STACKSIZE		256

/* Private macro -------------------------------------------------------------*/
#define hello 				"Hello!"
#define	CURSOR_STEP			0
/* Private variables ---------------------------------------------------------*/
USBD_HandleTypeDef USBD_Device;
uint8_t HID_Buffer[4];
static char print_buf[80];

/* Private function prototypes -----------------------------------------------*/
static void GetPointerData(uint8_t *pbuf);

/* Private functions ---------------------------------------------------------*/
void SystemClock_Config(void);
void HID_Mouse_Task(void *pArg);
void Display_Task(void *pArg);



/**
  * @brief  Main program
  * @param  None
  * @retval None
  */
int main(void)
{

	/* Configure the System clock to 64 MHz */
	SystemClock_Config();
	/* Set USB prescaler = 1 */
	__HAL_RCC_USB_CONFIG(1 << 22);
	//SystemCoreClock = SystemCoreClock;

	/* STM32F103xB HAL library initialization */
	HAL_Init();

	Board_Init();
	Debug_Console_Init();
	Adc_Init();

	xTaskCreate(gps_task, "GPS", GPS_TASK_STACKSIZE, NULL, GPS_TASK_PRIO, NULL);
	xTaskCreate(gsm_rx_task, "GSM_RX", GSM_RX_TASK_STACKSIZE, NULL, GSM_RX_TASK_PRIO, NULL);
#if GSM_DEBUG == 1
	xTaskCreate(gsm_debug_task, "GSM_DEBUG", GSM_DEBUG_TASK_STACKSIZE, NULL, GSM_DEBUG_TASK_PRIO, NULL);
#else
	xTaskCreate(message_task, "MESSAGE", MESSAGE_TASK_STACKSIZE, NULL, MESSAGE_TASK_PRIORITY, NULL);
	xTaskCreate(log_task, "LOG", LOG_TASK_STACKSIZE, NULL, LOG_TASK_PRIORITY, NULL);
	xTaskCreate(Display_Task, "Display",	DISPLAY_TASK_STACKSIZE, NULL,	DISPLAY_TASK_PRIORITY, NULL);
#endif
	xTaskCreate(monitor_task, "MONITOR", MONITOR_TASK_STACKSIZE, NULL, MONITOR_TASK_PRIORITY, NULL);
	xTaskCreate(battery_task, "BATTERY", BATTERY_TASK_STACKSIZE, NULL, BATTERY_TASK_PRIORITY, NULL);
	//xTaskCreate(HID_Mouse_Task, "USB",	256, NULL,	USB_TASK_PRIORITY, NULL);

	vTaskStartScheduler();

	/* We should never get here as control is now taken by the scheduler */
	for (;;);

}


void Display_Task(void *pArg)
{
	uint16_t 	adc_val;
	uint32_t 	len;

	while(1)
	{
#if 0
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
		vTaskDelay(2000);
	}

}

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
  RCC_ClkInitTypeDef clkinitstruct = {0};
  RCC_OscInitTypeDef oscinitstruct = {0};
  
  /* Configure PLL ------------------------------------------------------*/
  /* PLL configuration: PLLCLK = (HSI / 2) * PLLMUL = (8 / 2) * 16 = 64 MHz */
  /* PREDIV1 configuration: PREDIV1CLK = PLLCLK / HSEPredivValue = 64 / 1 = 64 MHz */
  /* Enable HSI and activate PLL with HSi_DIV2 as source */
  oscinitstruct.OscillatorType  = RCC_OSCILLATORTYPE_HSE;
  oscinitstruct.HSEState        = RCC_HSE_ON;
  oscinitstruct.LSEState        = RCC_LSE_OFF;
  oscinitstruct.HSIState        = RCC_HSI_OFF;
  oscinitstruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  oscinitstruct.HSEPredivValue    = RCC_HSE_PREDIV_DIV1;
  oscinitstruct.PLL.PLLState    = RCC_PLL_ON;
  oscinitstruct.PLL.PLLSource   = RCC_PLLSOURCE_HSE;
  oscinitstruct.PLL.PLLMUL      = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&oscinitstruct)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }

  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
     clocks dividers */
  clkinitstruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  clkinitstruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  clkinitstruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  clkinitstruct.APB2CLKDivider = RCC_HCLK_DIV1;
  clkinitstruct.APB1CLKDivider = RCC_HCLK_DIV2;  
  if (HAL_RCC_ClockConfig(&clkinitstruct, FLASH_LATENCY_2)!= HAL_OK)
  {
    /* Initialization Error */
    while(1); 
  }
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


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {}
}
#endif
