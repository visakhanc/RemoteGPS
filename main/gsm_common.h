/**
 * @file gsm_common.h
 * @date July 5, 2015
 * @author Visakhan
 */

#ifndef SOURCES_GSM_COMMON_H_
#define SOURCES_GSM_COMMON_H_

#include <stdbool.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "event_groups.h"
#include "timers.h"



#define HTTP_BUF_SIZE			512

/**
 * @defgroup GSM_Event_Bits Event Bit definition for various GSM related events
 * @{
 */
#define EVENT_GSM_OK			(1 << 0)	/**< GSM Command response is: OK */
#define EVENT_GSM_ERROR			(1 << 1)	/**< GSM Command response is: ERROR */
#define EVENT_GSM_RING			(1 << 2)	/**< GSM RING URC received */
#define EVENT_GSM_CLIP			(1 << 3)	/**< GSM CLIP URC received */
#define EVENT_GSM_HTTPREAD		(1 << 4)	/**< HTTP response data is read into buffer */
#define EVENT_GSM_HTTPCONNECT	(1 << 5)	/**< "CONNECT" response received - Module is ready to send or accept further data */
#define EVENT_GSM_CMTI			(1 << 6)
#define EVENT_GSM_CREG			(1 << 7)	/**< GSM module is registered */
#define EVENT_GSM_CMGS			(1 << 8)
#define EVENT_GSM_CMGR			(1 << 9)
#define EVENT_GSM_POWERSWITCH	(1 << 10)	/**< Switch is pressed so as to Power off the module */
#define EVENT_GSM_SENDMSG		(1 << 11)	/**< Indicates that an SMS need to be sent */
#define EVENT_GSM_CME_ERROR		(1 << 12)	/**< "CME ERROR" response received for a command */
#define EVENT_BUTTON_PRESS		(1 << 13)	/**< Switch is pressed */

/**
 * @}
 */

/**
 * @brief Various power states of GSM modules
 */
enum gsm_power_state {
	GSM_POWERDOWN = 0,//!< GSM Powerdown state
	GSM_POWERON = 1,  //!< GSM Normal state
	GSM_SLEEP = 2     //!< GSM Sleep state
};

/**
 * @brief Structure containing various status of GSM module
 */
typedef struct _gsm_status
{
	volatile enum gsm_power_state power_state;	/**< Present power state of GSM module */
	volatile bool registered;					/**< GSM module registered? */
	volatile bool sim_present;					/**< SIM card present? */
	volatile bool gprs_context;
	//volatile int cid;
	volatile int http_status;
	volatile int http_recv_len;					/**< Size of HTTP response data received in buffer */
	volatile bool http_sendto_module;			/**< When "CONNECT" is received: if true, set CONNECT event; if false, receive next UART data to http buffer */
	volatile bool http_overflow;				/**< HTTP buffer is full with received data */
	//volatile int http_read_count;
	uint8_t caller[15];							/**< Caller ID of Caller */
	int cme_error;								/**< Error number of CME Error */
} gsm_status_struct, *gsm_status_struct_ptr;


/* Globals */
extern gsm_status_struct gsm_status;

/* Public Function declarations */
extern int gsm_send_command(const char *cmd);
extern int gsm_uart_acquire(void);
extern int gsm_uart_release(void);
extern int gsm_uart_send(const char *data, uint32_t len);
extern void	gsm_set_event(uint32_t events);
extern void gsm_clear_event(uint32_t events);
extern uint32_t gsm_wait_for_event(uint32_t events, uint32_t delay_ticks);
extern int gsm_send_sms(const char *buf, int length, const char *address);

#endif /* SOURCES_GSM_COMMON_H_ */
