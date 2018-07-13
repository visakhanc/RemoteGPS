/**
 * @file	stm32f1_uart.h
 * @brief	Interrupt driven driver for STM32F1xx UART module
 * @author	Visakhan
 * @date	April 8, 2017
 */

#ifndef STM32F1_UART_H_
#define STM32F1_UART_H_

#include "stm32f1xx.h"
#include <stdbool.h>

#define UART_PARITY_NONE                    ((uint32_t)0x00000000)
#define UART_PARITY_EVEN                    ((uint32_t)USART_CR1_PCE)
#define UART_PARITY_ODD                     ((uint32_t)(USART_CR1_PCE | USART_CR1_PS))

#define UART_STOPBITS_1                     ((uint32_t)0x00000000)
#define UART_STOPBITS_2                     ((uint32_t)USART_CR2_STOP_1)


#define UART_DIV_SAMPLING16(_PCLK_, _BAUD_)         (((_PCLK_)*25)/(4*(_BAUD_)))
#define UART_DIVMANT_SAMPLING16(_PCLK_, _BAUD_)     (UART_DIV_SAMPLING16((_PCLK_), (_BAUD_))/100)
#define UART_DIVFRAQ_SAMPLING16(_PCLK_, _BAUD_)     (((UART_DIV_SAMPLING16((_PCLK_), (_BAUD_)) - (UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) * 100)) * 16 + 50) / 100)
/* UART BRR = mantissa + overflow + fraction
            = (UART DIVMANT << 4) + (UART DIVFRAQ & 0xF0) + (UART DIVFRAQ & 0x0F) */
#define UART_BRR_SAMPLING16(_PCLK_, _BAUD_)            (((UART_DIVMANT_SAMPLING16((_PCLK_), (_BAUD_)) << 4) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0xF0)) + \
                                                        (UART_DIVFRAQ_SAMPLING16((_PCLK_), (_BAUD_)) & 0x0F))

/**
 * @brief UART Parity values
 */
typedef enum {
	Uart_Parity_None = UART_PARITY_NONE,//!<  No parity
	Uart_Parity_Odd = UART_PARITY_ODD,  //!<  Odd parity
	Uart_Parity_Even = UART_PARITY_EVEN //!<  Even parity
} uart_parity_mode_t;

/**
 * UART Stop bit values
 */
typedef enum {
	Uart_StopBits_1 = UART_STOPBITS_1,//!<  One stop bit
	Uart_StopBits_2 = UART_STOPBITS_2 //!<  Two stop bits
} uart_stopbit_mode_t;

/**
 * @brief UART operation status updated by UART ISR
 */
typedef enum {
	Uart_Status_TxDone = 0,//!<  UART Transmit operation was completed
	Uart_Status_RxDone,    //!<  UART Receive operation was completed
	Uart_Status_Error      //!<  An error occurred in UART operation
} uart_status_t;

/**
 * @brief Transmit status of UART
 */
typedef enum {
	Uart_Tx_Ready = 0,//!<  Ready to Transmit
	Uart_Tx_Busy,     //!<  Currently transmitting
	Uart_Tx_Error     //!<  Transmission Error
} uart_tx_status_t;

/**
 * @brief Receive status of UART
 */
typedef enum {
	Uart_Rx_Ready = 0,//!<  Ready to receive
	Uart_Rx_Busy,     //!<  Currently receiving
	Uart_Rx_Error     //!<  Receive error
} uart_rx_status_t;

typedef struct uart_handle uart_handle_t;

typedef void (*uart_callback_t)(USART_TypeDef *base, uart_handle_t *handle, uart_status_t status);

/**
 * @brief Handle structure definition for UART driver
 */
struct uart_handle {
	volatile bool initialized;	/*!< Driver initialized? */
	volatile uint8_t *RxBuf;	/*!< Buffer to store received characters */
	volatile uint32_t RxSize;	/*!< Number of characters to receive */
	volatile uint8_t *TxBuf;	/*!< Buffer of characters to be transmitted */
	volatile uint32_t TxSize;	/*!< Number of characters to be transmitted */
	uart_callback_t callback;	/*!< Callback function to be called after completion of Transmit/Receive operation */
	volatile uart_tx_status_t TxState;	/*!< Transmitter state of UART */
	volatile uart_rx_status_t RxState;	/*!< Receiver state of UART */
};

/**
 * @brief Structure definition for UART initialization parameters
 */
typedef struct {
	uint32_t BaudRate;				/*!< Baud rate in bps */
	uart_parity_mode_t ParityMode; 	/*!< Parity mode */
	uart_stopbit_mode_t StopBits;	/*!< Stop bits */
	bool EnableTx;					/*!< Enable Transmitter? */
	bool EnableRx;					/*!< Enable Receiver? */
} uart_config_t;


/**
 * @brief Driver function to initialize UART module with the given configuration.
 * @param base UART base address
 * @param handle UART driver handle structure
 * @param config UART initialization configuration
 * @return Returns 0: Success, 1: Error
 */
uint32_t Uart_Init(USART_TypeDef *base, uart_handle_t *handle, uart_config_t *config);


/**
 * @brief Sets the callback function to be called by the driver on completion of Transmit/Receive operation
 * @param base UART base address
 * @param callback Callback function
 * @return Returns 0: Success, 1: Error
 */
uint32_t Uart_Set_Callback(USART_TypeDef *base, uart_callback_t callback);



/**
 * @brief Driver function to transmit the characters in the buffer through UART.
 *        Returns immediately without blocking. The caller needs to wait until callback function
 *        is called with a status indicating Transmit completion
 * @param base UART Base Address
 * @param data Buffer containing data to be sent
 * @param size Number of character to be sent
 * @return : Returns 0 : Success, 1 : Error
 */
uint32_t Uart_Send(USART_TypeDef *base, uint8_t *data, uint32_t size);



/**
 * @brief Driver function to start receiving data by enabling Rx interrupt for the UART.
 *        Rx Buffer and size should be set explicitly by @ref Uart_Set_Rx_Params before calling this function
 * @param base UART base address
 * @return Always returns 0
 */
uint32_t Uart_StartReceive(USART_TypeDef *base);


/**
 * @brief Stops reception of data for the UART by disabling Rx interrupt
 * @param base UART base address
 * @return Always returns 0
 */
uint32_t Uart_StopReceive(USART_TypeDef *base);



/**
 * @brief Set UART Rx buffer and size parameters
 * @param base UART base address
 * @param buf Buffer to store received characters by the driver
 * @param size Size of buffer in bytes
 * @return 0:success, 1:Error
 */
uint32_t Uart_Set_Rx_Params(USART_TypeDef *base, uint8_t *buf, uint32_t size);



/**
 * @brief Driver function to receive characters from UART to a buffer
 * @param base UART base address
 * @param buf Buffer to store received characters
 * @param size Size of buffer
 * @return Returns 0: Success, 1: Error
 */
uint32_t Uart_Receive(USART_TypeDef *base, uint8_t *buf, uint32_t size);



uint32_t Uart_DisableRx(USART_TypeDef *base);
uint32_t Uart_EnableRx(USART_TypeDef *base);

#endif /* STM32F1_UART_H_ */
