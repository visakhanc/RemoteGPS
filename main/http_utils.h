/*
 * http_utils.h
 *
 *  Created on: Jun 17, 2016
 *  Modified: Apr 21, 2017
 *  	- Modified for STM32F103 target / M66 GSM module
 *  Author: Visakhan C
 */

#ifndef SOURCE_HTTP_UTILS_H_
#define SOURCE_HTTP_UTILS_H_

#include <stdint.h>


typedef enum {
	HTTP_OK = 0,
	HTTP_ERR_URL_TIMOUT = 0x1000,
	HTTP_ERR_URL_CMD = 0x1001,
	HTTP_ERR_READ = 0x1002,
	HTTP_ERR_OVERFLOW = 0x1003,
	HTTP_ERR_POST = 0x1004,
	HTTP_ERR_EVENT_TIMEOUT = 0x1FFF
} http_error_t;

extern http_error_t http_get(char *url,  uint32_t url_len);
extern http_error_t http_post(char *url, uint32_t urlLen, uint8_t *data, uint32_t dataLen);

#if 0
extern int http_open_context(void);
extern int http_close_context(void);
extern int http_init(void);
extern int http_terminate(void);
int http_read(uint8_t *buf, int offset, int size);
extern int http_find_string(const char* str, uint8_t *page_buf, int bufsize);
extern int gprs_configure(void);
#endif

#endif /* SOURCE_HTTP_UTILS_H_ */
