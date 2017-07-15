/**
 * @file	stm32f1_uart.c
 * @brief	Interrupt driven driver for STM32F1xx UART module
 * @author	Visakhan
 * @date	April 8, 2017
 */


#include "stm32f1_uart.h"
#include "stm32f1xx_hal.h"

static USART_TypeDef *UartBasePtrs[] = {USART1, USART2, USART3};
#define UART_COUNT (sizeof(UartBasePtrs)/sizeof(UartBasePtrs[0]))
static uart_handle_t *s_UartHandles[UART_COUNT];


/**
 * @brief Get the UART instance corresponding to the UART Base address
 * @param base : Base address of UART register structure
 * @return UART instance number (0,1...)
 */
static uint32_t Uart_Get_Instance(USART_TypeDef *base)
{
	uint32_t i;
	for(i = 0; i < UART_COUNT; i++)
	{
		if(base == UartBasePtrs[i])
			break;
	}
	return i;
}



uint32_t Uart_Init(USART_TypeDef *base, uart_handle_t *handle, uart_config_t *config)
{
	uint32_t instance;

	/* UART peripheral clock and Tx, Rx pin port clocks should be enabled already */

	/* Get UART instance */
	instance = Uart_Get_Instance(base);
	if(instance >=  UART_COUNT) {
		return 1;
	}

	/* Disable UART */
	base->CR1 &= ~USART_CR1_UE;

	/* Set Configuration */
	base->CR2 &= ~(USART_CR2_STOP);
	base->CR2 |= config->StopBits;
	base->CR1 &= ~(USART_CR1_M | USART_CR1_PCE | USART_CR1_PS | USART_CR1_TE | USART_CR1_RE);
	base->CR1 |= config->ParityMode;
	if(config->EnableRx) {
		base->CR1 |= USART_CR1_RE;
	}
	if(config->EnableTx) {
		base->CR1 |= USART_CR1_TE;
	}
	if((base == USART1))
	{
		base->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK2Freq(), config->BaudRate);
	}
	else
	{
		base->BRR = UART_BRR_SAMPLING16(HAL_RCC_GetPCLK1Freq(), config->BaudRate);
	}

	/* Enable UART */
	base->CR1 |= USART_CR1_UE;
	handle->TxState = Uart_Tx_Ready;
	handle->RxState = Uart_Rx_Ready;
	handle->initialized = true;
	/* Copy handle pointer */
	s_UartHandles[instance] = handle;

	return 0;
}

uint32_t Uart_Set_Callback(USART_TypeDef *base, uart_callback_t callback)
{
	uint32_t instance = Uart_Get_Instance(base);
	if(instance >= UART_COUNT) {
		return 1;
	}
	s_UartHandles[instance]->callback = callback;
	return 0;
}



uint32_t Uart_Send(USART_TypeDef *base, uint8_t *data, uint32_t size)
{
	uint32_t instance = Uart_Get_Instance(base);
	uart_handle_t *handle;

	if(instance >= UART_COUNT) {
		return 1;
	}
	if(size == 0) {
		return 1;
	}
	/* Make sure handle is initialized and Tx is not busy */
	handle = s_UartHandles[instance];
	if((handle->initialized != true) || (handle->TxState == Uart_Tx_Busy)) {
		return 1;
	}
	/* Set transmit data details in handle */
	handle->TxBuf = data;
	handle->TxSize = size;

	handle->TxState = Uart_Tx_Busy;

	/* Enable Transmit Empty interrupt */
	base->CR1 |= USART_CR1_TXEIE;

	return 0;
}



uint32_t Uart_StartReceive(USART_TypeDef *base)
{
	/* Enable Receive Not Empty interrupt */
	base->CR1 |= USART_CR1_RXNEIE;
	/* Enable Receive Error interrupts */
	base->CR3 |= USART_CR3_EIE;

	return 0;
}




uint32_t Uart_StopReceive(USART_TypeDef *base)
{
	/* Disable Receive Not Empty interrupt */
	base->CR1 &= ~USART_CR1_RXNEIE;
	/* Disable Receive Error interrupts */
	base->CR3 &= ~USART_CR3_EIE;

	return 0;
}



uint32_t Uart_Set_Rx_Params(USART_TypeDef *base, uint8_t *buf, uint32_t size)
{
	uint32_t instance = Uart_Get_Instance(base);
	uart_handle_t * handle;
	if(instance >= UART_COUNT) {
		return 1;
	}
	if(size == 0) {
		return 1;
	}
	handle = s_UartHandles[instance];
	handle->RxBuf = buf;
	handle->RxSize = size;
	return 0;
}



uint32_t Uart_Receive(USART_TypeDef *base, uint8_t *buf, uint32_t size)
{
	uint32_t instance = Uart_Get_Instance(base);
	uart_handle_t *handle;

	if(instance >= UART_COUNT) {
		return 1;
	}
	if(size == 0) {
		return 1;
	}
	handle = s_UartHandles[instance];
	/* Make sure handle is initialized and Rx is not busy */
	if((handle->initialized != true) || (handle->RxState == Uart_Rx_Busy)) {
		return 1;
	}
	handle->RxBuf = buf;
	handle->RxSize = size;
	handle->RxState = Uart_Rx_Busy;

	Uart_StartReceive(base);

	return 0;
}

uint32_t Uart_DisableRx(USART_TypeDef *base)
{
	return 0;

}

uint32_t Uart_EnableRx(USART_TypeDef *base)
{
	return 0;

}

static void UART_Rx_Tx_Handler(USART_TypeDef *base, uart_handle_t *handle)
{
	//TODO: Handle concurrent access (through mutex or TxState?)
	/* UART Transmit Register Empty Interrupt */
	if((base->CR1 & USART_CR1_TXEIE) && (base->SR & USART_SR_TXE)) {
		/* Transmit byte from buffer */
		base->DR = *handle->TxBuf++;
		if(--handle->TxSize == 0) {
			/* Disable TXE interrupt */
			base->CR1 &= ~USART_CR1_TXEIE;
			/* Enable TC interrupt */
			base->CR1 |= USART_CR1_TCIE;
		}
	}

	/* UART Transmit complete interrupt */
	if((base->CR1 & USART_CR1_TCIE) && (base->SR & USART_SR_TC)) {
		/* Disable TC interrupt to finish transmission */
		base->CR1 &= ~USART_CR1_TCIE;
		handle->TxState = Uart_Tx_Ready;
		/* Callback */
		if(handle->callback) {
			handle->callback(base, handle, Uart_Status_TxDone);
		}
	}
	/* UART Receive Not Empty interrupt */
	if((base->CR1 & USART_CR1_RXNEIE) && (base->SR & USART_SR_RXNE)) {
		/* Receive data to buffer */
		*handle->RxBuf++ = (uint8_t)base->DR;
		if(--handle->RxSize == 0) {
			if(handle->callback) {
				handle->callback(base, handle, Uart_Status_RxDone);
			}
		}
		/* Disable Receive interrupt to finish Rx transfer */
		if(handle->RxSize == 0) {
			base->CR1 &= ~USART_CR1_RXNEIE;
			base->CR3 &= ~USART_CR3_EIE;
			handle->RxState = Uart_Rx_Ready;
		}
	}

}

void USART1_IRQHandler(void)
{
	UART_Rx_Tx_Handler(USART1, s_UartHandles[0]);
}

void USART2_IRQHandler(void)
{
	UART_Rx_Tx_Handler(USART2, s_UartHandles[1]);
}

void USART3_IRQHandler(void)
{
	UART_Rx_Tx_Handler(USART3, s_UartHandles[2]);
}
