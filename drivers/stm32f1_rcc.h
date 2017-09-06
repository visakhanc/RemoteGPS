/**
 * @file	stm32f1_rcc.h
 * @brief	Driver for Reset & Clock Control module of STM32F103
 * @author	Visakhan
 * @date	June 8, 2017
 */

#ifndef STM32F1_RCC_H_
#define STM32F1_RCC_H_


#include <stdbool.h>
#include "stm32f1xx_hal_rcc.h"		/* To use existing macros and definitions from HAL driver */

/* Driver functions */

/**
 * @brief Function to initialize High-Speed Oscillator(HSE)
 * @param   [in] prediv True if HSE clock has to be divided by 2 before PLL
 * @retval	None
 */
void RCC_HSE_OscInit(bool prediv);


/**
 * @brief Disable the HSE Clock
 * @attention HSE clock cannot be disabled when HSE or HSE through PLL is currently being used as System clock.
 * In this case, switch system clock to use HSI clock, before disabling HSE clock.
 * @retval None
 */
void RCC_HSE_Disable(void);


/**
 * @brief Enables the HSI clock
 * @retval None
 */
void RCC_HSI_Enable(void);



/**
 * @brief Disable the HSI clock
 * @attention HSI clock cannot be disabled when HSI or HSI through PLL is currently being used as System clock.
 * In this case, switch system clock to use HSE clock, before disabling HSI clock.
 * @retval None
 */
void RCC_HSI_Disable(void);


/**
 * @brief 	Turn on PLL with given input source multiplied by given factor
 * @attention Cannot be called when PLL is already being used for generating System clock
 * @param	[in] pll_src Input source for PLL. See @ref RCC_PLL_Clock_Source for possible values
 * @param 	[in] pll_mul Multiplication factor for PLL. See @ref RCCEx_PLL_Multiplication_Factor for possible values
 * @retval 	None
 */
void RCC_PLL_Init(uint32_t pll_src, uint32_t pll_mul);


/**
 * @brief Setup the System Clock frequency from given source clock
 * @note The dividers are set as follows: HCLK_DIV = 1, PCLK2_DIV = 1, PCLK1_DIV = 2
 * @attention Ensure that the Clock source to be used is ready before calling this function
 * @param sysclk_src Clock source to be selected as System clock. See @ref RCC_System_Clock_Source
 * for possible values
 * @retval None
 */
void RCC_Clock_Config(uint32_t sysclk_src);


/**
  * @brief  Returns the HCLK frequency
  * @note   Each time HCLK changes, this function must be called to update the
  *         right HCLK value. Otherwise, any configuration based on this function will be incorrect.
  * @note   The SystemCoreClock CMSIS variable is used to store System Clock Frequency
  * @retval HCLK frequency
  */
uint32_t RCC_GetHCLKFreq(void);

/**
  * @brief  Returns the PCLK1 frequency
  * @note   Each time PCLK1 changes, this function must be called to update the
  *         right PCLK1 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK1 frequency
  */
uint32_t RCC_GetPCLK1Freq(void);



/**
  * @brief  Returns the PCLK2 frequency
  * @note   Each time PCLK2 changes, this function must be called to update the
  *         right PCLK2 value. Otherwise, any configuration based on this function will be incorrect.
  * @retval PCLK2 frequency
  */
uint32_t RCC_GetPCLK2Freq(void);


#endif /* STM32F1_RCC_H_ */
