/*
 * circular_buf.c
 *
 *  Created on: Apr 9, 2017
 *      Author: Visakhan C
 */

#include <stdio.h>
#include "circular_buf.h"


void cb_display(circbuf_t *handle)
{
	int i;
	
	printf("\n");
	for(i = 0; i < handle->size; i++) {
		if(i == handle->r_index) {
			putchar('V');
		}
		putchar('\t');
	}
	printf("\n");
	for(i = 0; i < handle->size; i++) {
		printf("%c\t", handle->buf[i]);
	}
	printf("\n");
	for(i = 0; i < handle->size; i++) {
		if(i == handle->w_index) {
			putchar('^');
		}
		putchar('\t');
	}
	printf("\n");
}


void cb_init(circbuf_t *handle, uint8_t *buf, uint32_t size, bool overwrite)
{	
	handle->buf = buf;
	handle->size = size;
	handle->r_index = 0;
	handle->w_index = 0;
	handle->overwrite = overwrite;
}


uint32_t cb_write(circbuf_t *handle, uint8_t data)
{
	uint32_t cur = handle->w_index;
	uint32_t next = (cur + 1) % (handle->size);
	
	/* Write data to buffer */
	handle->buf[cur] = data;
	/* If buffer is full, overwrite old data if configured so */
	if(next == handle->r_index) { 
		if(handle->overwrite == true) {
			handle->w_index = next;
			handle->r_index = (next + 1) % (handle->size);
		}
		return 1;
	}
	handle->w_index = next;
	return 0; 
}

uint32_t cb_read(circbuf_t *handle, uint8_t *data)
{
	uint32_t cur = handle->r_index;
	/* If no data to read, return */
	if(cur == handle->w_index) {
		return 1;
	}
	*data = handle->buf[cur];
	handle->r_index = (cur + 1) % (handle->size);
	return 0;
}

uint32_t cb_count(circbuf_t *handle)
{
	uint32_t r = handle->r_index;
	uint32_t w = handle->w_index;
	return (r <= w) ? (w - r) : (handle->size - (r - w));
}
