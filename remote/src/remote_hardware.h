/*
 * remote_hardware.h
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */

#ifndef REMOTE_HARDWARE_H_
#define REMOTE_HARDWARE_H_

#include "avr/io.h"

uint8_t GetADC5(void);
uint8_t GetADC6(void);
uint8_t GetButton(void);
uint8_t GetJoyState(void);
uint8_t isDebounceDone(void);
//uint8_t noButtonsPressed(void);
uint8_t isTimeOut(void);
void resetTimeOut(void);
//void clrDebounceFlag(void);
uint8_t hasButtonChanged(void);
void clrButtonChanged(void);
void initHardware(void);
void setLED(void);
void clrLED(void);
void tglLED(void);

#endif /* REMOTE_HARDWARE_H_ */
