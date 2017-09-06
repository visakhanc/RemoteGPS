/*
 * debug_console.h
 *
 *  Created on: Apr 9, 2017
 *      Author: Visakhan C
 */

#ifndef DEBUG_CONSOLE_H_
#define DEBUG_CONSOLE_H_

#include <stdbool.h>
#include <string.h>
#include "board.h"

#define DEBUG_PUTS(x)					Debug_Console_PutBuf((char *)(x), strlen(x))

extern uint8_t __print_buf[];
uint32_t Debug_Console_Init(void);
uint8_t Debug_Console_GetChar(void);
uint32_t Debug_Console_PutChar(uint8_t ch);
uint32_t Debug_Console_PutBuf(char *buf, uint32_t size);

#endif /* DEBUG_CONSOLE_H_ */
