/*
 * util.h
 *
 *  Created on: Nov 1, 2010
 *      Author: grant
 */

#ifndef UTIL_H_
#define UTIL_H_

#ifndef NULL
#define NULL ((void *)0)
#endif

void dbgLEDinit(void);
void dbgLEDset(void);
void dbgLEDclr(void);
void dbgLEDtgl(void);
void dbgUSARTinit(void);
void dbgPutChar(char c);
void dbgPutStr(char *str);

#endif /* UTIL_H_ */
