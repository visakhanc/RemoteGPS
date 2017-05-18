/*
 * circular_buf.h
 *
 *  Created on: Apr 9, 2017
 *      Author: Visakhan C
 */

#ifndef CIRCULAR_BUF_H
#define CIRCULAR_BUF_H

#include <stdbool.h>
#include <stdint.h>

typedef struct circ_buf_handle {
	volatile uint32_t w_index;
	volatile uint32_t r_index;
	uint32_t size;
	uint8_t *buf;
	bool overwrite;
} circbuf_t;


/* Initialize Circular buffer
	Parameters:
	handle : pointer to a circbuf_t structure
	buf : buffer location
	size : size of buffer
	overwrite : true - overwrite old data in case of overflow
				false - keep old data in case of overflow
*/
void cb_init(circbuf_t *handle, uint8_t *buf, uint32_t size, bool overwrite);

/* Write an element to circular buffer 
 * 	Write to current location, increment pointer if next is not read pointer
 * 	Returns 0: success 1: Buffer full/overwritten old data
 */
uint32_t cb_write(circbuf_t *handle, uint8_t data);

/* Read an element from circular buffer 
 * 	If current location is write pointer, return error
 *  Otherwise, read element and increment pointer
 * 	Returns 0: success 1: empty 
 */
uint32_t cb_read(circbuf_t *handle, uint8_t *p_data);

/* Display */
void cb_display(circbuf_t *handle);

/* Returns the present number of items in the buffer */
uint32_t cb_count(circbuf_t *handle);

#endif
