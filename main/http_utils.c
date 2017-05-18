/*
 * http_utils.c
 *
 *  Created on: Jul 11, 2015
 *	Modified: Apr 21, 2017
 *		- Modified for STM32F103 target / M66 GSM module
 *
 *  Author: Visakhan C
 */

#include <stdio.h>
#include <string.h>
#include "http_utils.h"
#include "gsm_common.h"
#include "debug_console.h"


//extern char http_buf[];

static char http_cmd_buf[80];
//volatile static bool http_initialized = false;

/* Description 	:	Use M66 HTTP GET feature to send GET request to given URL
 * Arguments 	: 	url - Null terminated URL string
 * 					urlLen - Length of URL
 * Returns 		: 	Success (0)
 * 					Any of the error codes
 */
http_error_t http_get(char *url, uint32_t urlLen)
{
	int ret = 0;
	EventBits_t ev;

	gsm_status.cme_error = 0;
	/* Send URL to module */
	snprintf(http_cmd_buf, sizeof(http_cmd_buf), "AT+QHTTPURL=%d,10", (int)urlLen);
	gsm_status.http_sendto_module = true;
	gsm_send_command(http_cmd_buf);
	ev = gsm_wait_for_event(EVENT_GSM_HTTPCONNECT|EVENT_GSM_ERROR, 3000);
	if(ev & EVENT_GSM_HTTPCONNECT) {
		DEBUG_PUTS("Sending URL\r\n");
		gsm_send_command(url);
		ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_CME_ERROR, 3000);
		if(!(ev & EVENT_GSM_OK)) {
			ret = (gsm_status.cme_error == 0) ? HTTP_ERR_EVENT_TIMEOUT : gsm_status.cme_error;
		}
	}
	else if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("URL command error\r\n");
		//ret = HTTP_ERR_URL;
		ret = gsm_status.cme_error;
	}
	else {
		//ret = HTTP_ERR_EVENT_TIMEOUT;
		ret = 2;
	}

	if(0 == ret) {
		/* Issue GET command */
		DEBUG_PUTS("GET...\r\n");
		gsm_send_command("AT+QHTTPGET=20");
		ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 30*1000);
		if(ev & EVENT_GSM_ERROR) {
			DEBUG_PUTS("GET error\r\n");
			//ret = HTTP_ERR_GET;
			ret = gsm_status.cme_error;
		}
		else if(!(ev & EVENT_GSM_OK)) {
			ret = HTTP_ERR_EVENT_TIMEOUT;
		}
	}

	/* Read received data */
	if(0 == ret) {
		gsm_status.http_sendto_module = false;
		gsm_send_command("AT+QHTTPREAD=10");
		ev = gsm_wait_for_event(EVENT_GSM_HTTPREAD, 15*1000);
		if(ev & EVENT_GSM_HTTPREAD) {
			DEBUG_PUTS("GET OK\r\n");
		}
		else {
			DEBUG_PUTS("Read timeout\r\n");
			ret = HTTP_ERR_EVENT_TIMEOUT;
		}
	}
	return ret;
}



/* Description 	:	Use HTTP POST feature to send data to given URL
 * Arguments 	: 	url - Null terminated URL string
 * 					urlLen - Length of URL
 * 					data - pointer to data to be posted to URL
 * 					dataLen - length of data
 * Returns 		: 	Success (0) or
 * 					Any of the error codes
 */
http_error_t http_post(char *url, uint32_t urlLen, uint8_t *data, uint32_t dataLen)
{
	int ret = 0;
	EventBits_t ev;

	gsm_status.cme_error = 0;
	gsm_clear_event(EVENT_GSM_OK|EVENT_GSM_ERROR|EVENT_GSM_CME_ERROR);

	/* Send URL to module */
	snprintf(http_cmd_buf, sizeof(http_cmd_buf), "AT+QHTTPURL=%d,3", (int)urlLen);
	gsm_status.http_sendto_module = true;
	gsm_send_command(http_cmd_buf);
	ev = gsm_wait_for_event(EVENT_GSM_HTTPCONNECT|EVENT_GSM_ERROR|EVENT_GSM_CME_ERROR, 1000);
	if(ev & EVENT_GSM_HTTPCONNECT) {
		DEBUG_PUTS("Sending URL..\r\n");
		gsm_send_command(url);
		ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_CME_ERROR, 4000);
		if(!(ev & EVENT_GSM_OK)) {
			ret = (gsm_status.cme_error == 0) ? HTTP_ERR_EVENT_TIMEOUT : gsm_status.cme_error;
		}
	}
	else if(ev & (EVENT_GSM_ERROR|EVENT_GSM_CME_ERROR)) {
		DEBUG_PUTS("URL command error\r\n");
		ret = (gsm_status.cme_error) ? gsm_status.cme_error : HTTP_ERR_URL;
	}
	else {
		ret = HTTP_ERR_EVENT_TIMEOUT;
		//ret = 2;
	}

	if(!ret) {
		DEBUG_PUTS("Sending POST Req..\r\n");
		snprintf(http_cmd_buf, sizeof(http_cmd_buf), "AT+QHTTPPOST=%d,5,25", (int)dataLen);
		gsm_send_command(http_cmd_buf);
		ev = gsm_wait_for_event(EVENT_GSM_HTTPCONNECT|EVENT_GSM_CME_ERROR|EVENT_GSM_ERROR, 25000);
		if(ev & EVENT_GSM_HTTPCONNECT) {
			gsm_send_command((char *)data);
			ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_CME_ERROR, 6000);
			if(ev & EVENT_GSM_CME_ERROR) {
				DEBUG_PUTS("POST data error\r\n");
				ret = gsm_status.cme_error;
			}
			else if(!(ev & EVENT_GSM_OK)) {
				ret = HTTP_ERR_EVENT_TIMEOUT;
			}
		}
		else if(ev & (EVENT_GSM_CME_ERROR|EVENT_GSM_ERROR)) {
			DEBUG_PUTS("POST Req error\r\n");
			ret = gsm_status.cme_error ? gsm_status.cme_error : HTTP_ERR_POST;
		}
		else {
			ret = HTTP_ERR_EVENT_TIMEOUT;
		}
	}
	if(!ret) {
		/* Read received data */
		gsm_status.http_sendto_module = false;
		DEBUG_PUTS("Read..\r\n");
		gsm_send_command("AT+QHTTPREAD=20");
		ev = gsm_wait_for_event(EVENT_GSM_HTTPREAD|EVENT_GSM_CME_ERROR, 6000);
		if(ev & EVENT_GSM_HTTPREAD) {
			DEBUG_PUTS("POST OK\r\n");
		}
		else {
			DEBUG_PUTS("READ error\r\n");
			ret = gsm_status.cme_error ? gsm_status.cme_error : HTTP_ERR_EVENT_TIMEOUT;
		}
	}

	return ret;
}

#if 0
int http_open_context(void)
{
	EventBits_t ev;


	/* query gprs context */
	gsm_send_command("AT+SAPBR=2,1");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 2*1000);
	if(ev & EVENT_GSM_OK) {
		if(true == gsm_status.gprs_context)
		{
			DEBUG_PUTS("\n\rUsing already context");
		}
		else {
			DEBUG_PUTS("\n\ropening context");

			/* open gprs context */
			gsm_send_command("AT+SAPBR=1,1");
			ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 15*1000);
			if(ev & EVENT_GSM_ERROR) {
				DEBUG_PUTS("\n\rgprs open error");
				return 1;
			}
			else if(!(ev & EVENT_GSM_OK)) {
				DEBUG_PUTS("\n\rgprs open timeout");
				return 1;
			}
		}
	}
	else {
		DEBUG_PUTS("\n\rquery error/timout");
		return 1;
	}

	DEBUG_PUTS("\n\rgprs opened");
	return 0;
}


int http_close_context(void)
{
	EventBits_t ev;

	gsm_send_command("AT+SAPBR=2,1");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 2*1000);
	if(ev & EVENT_GSM_OK) {
		if(true == gsm_status.gprs_context) {
			/* close gprs context */
			gsm_send_command("AT+SAPBR=0,1");
			ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 5*1000);
			if(ev & EVENT_GSM_ERROR)
			{
				DEBUG_PUTS("\n\rgprs close error");
				return 1;
			}
			else if(!(ev & EVENT_GSM_OK)) {
				DEBUG_PUTS("\r\ngprs close timeout");
				return 1;
			}
		}
	}
	else {
		DEBUG_PUTS("\r\ngprs query error");
		return 1;
	}

	return 0;
}


int http_init(void)
{
	int ret = 0;
	EventBits_t ev;

	if(true == http_initialized) {
		return 0;
	}

	gsm_send_command("AT+HTTPINIT");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 2*1000);
	if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\n\rHTTPINIT error");
		ret = 1;
	}
	else if(!(ev & EVENT_GSM_OK)) {
		DEBUG_PUTS("\n\rhttp init timeout");
		ret = 1;
	}
	else {
		http_initialized = true;
	}

	/* Set context ID */
	gsm_send_command("AT+HTTPPARA=\"CID\",1");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 2*1000);
	if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\n\rERROR: CID error");
		ret = 1;
	}
	else if(!(ev & EVENT_GSM_OK)) {
		ret = 1;
		DEBUG_PUTS("\n\rERROR: CID timout");
	}

	return ret;
}


int http_terminate(void)
{
	int ret = 0;
	EventBits_t ev;

	if(false == http_initialized) {
		return 0;
	}

	/* close HTTP */
	gsm_send_command("AT+HTTPTERM");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 2000);
	if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\n\rhttpterm error");
		ret = 1;
	}
	else if(!(ev & EVENT_GSM_OK)) {
		DEBUG_PUTS("\r\nhttpterm timeout");
		ret = 1;
	}
	else {
		http_initialized = false;
	}

	return ret;
}


/* Read 'size' bytes from the currently received http page at 'offset' bytes
 * from SIM900 internal buffer, into the given buffer */
int http_read(uint8_t *buf, int offset, int size)
{
	EventBits_t ev;
	char cmd[25];

	if(size > HTTP_BUF_SIZE) {
		return 1;
	}

	snprintf(cmd, sizeof(cmd), "AT+HTTPREAD=%d,%d", offset, size);
	gsm_send_command(cmd);
	ev = gsm_wait_for_event(EVENT_GSM_HTTPREAD|EVENT_GSM_ERROR, 2*1000);
	if(!(ev & (EVENT_GSM_HTTPREAD|EVENT_GSM_ERROR)))
	{
		DEBUG_PUTS("\n\rRead error");
		return 1;
	}
	else {
		memcpy(buf, http_buf, size);
	}

	gsm_wait_for_event(EVENT_GSM_OK, 1000);

	return 0;
}


/* Look for the given string in the received HTTP page in SIM900 internal buffer
 * If found, 'page_buf is filled with content from the received page, starting with
 * given string upto either end of the page or the size of page_buf ('bufsize')
 */
int http_find_string(const char* str, uint8_t *page_buf, int bufsize)
{
	int total_size = gsm_status.http_recv_len;
	int get_size = bufsize - 1;
	int page_offset = 0;
	int i;
	char *ptr;

	do {

		if(total_size < get_size) {
			get_size = total_size;
		}

		/* get some chars */
		http_read(&page_buf[bufsize - get_size - 1], page_offset, get_size);
		page_buf[bufsize-1] = '\0';
		page_offset += get_size;
		total_size -= get_size;

		/* search for it */
		ptr = strstr(page_buf, str);

		/* found! copy all chars in buffer starting at search string */
		if(NULL != ptr)
		{
			i = 0;
			while(ptr[i] != '\0') {
				page_buf[i] = ptr[i];
				i++;
			}

			/* copy more characters to fill the buffer */
			get_size = (total_size < (bufsize - i)) ? total_size : (bufsize - i);
			http_read(&page_buf[i], page_offset, get_size-1);

			page_buf[i+get_size] = '\0';
			return 0;
		}

		/* move the last few characters in the buffer to start of buffer */
		strncpy(page_buf, &page_buf[bufsize - strlen(str) - 1], strlen(str));

		/* continue searching */
		get_size = bufsize - strlen(str) - 1;

	} while (total_size > 0);

	return -1;
}

/*
 * Configure GPRS for first time
 */
int gprs_configure(void)
{
	EventBits_t ev;

	/* GPRS attach */
	gsm_send_command("AT+CGATT=1");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 2*1000);
	if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\n\rCGATT error");
		return 1;
	}
	else if(!(ev & EVENT_GSM_OK)) {
		DEBUG_PUTS("\r\nCGATT timeout");
	}

	gsm_send_command("AT+SAPBR=3,1,\"CONTYPE\",\"GPRS\"");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 2*1000);
	if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\n\rCONTYPE error");
		return 1;
	}
	else if(!(ev & EVENT_GSM_OK)) {
		DEBUG_PUTS("\r\nCONTYPE timeout");
	}

	gsm_send_command("AT+SAPBR=3,1,\"APN\",\"www\"");
	ev = gsm_wait_for_event(EVENT_GSM_OK|EVENT_GSM_ERROR, 2*1000);
	if(ev & EVENT_GSM_ERROR) {
		DEBUG_PUTS("\n\rAPN error");
		return 1;
	}
	else if(!(ev & EVENT_GSM_OK)) {
		DEBUG_PUTS("\n\rAPN timeout");
		return 1;
	}

	return 0;

}

#endif

