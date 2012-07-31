/*
 * remote_hardware.h
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */

#ifndef REMOTE_HARDWARE_H_
#define REMOTE_HARDWARE_H_

#include <avr/io.h>

//#define INSTRUCTOR_REMOTE
#define STUDENT_JOYSTICK

#ifndef INSTRUCTOR_REMOTE
#ifndef STUDENT_JOYSTICK
#error "Please #define either INSTRUCTOR_REMOTE or WIRELESS_JOYSTICK in remote_hardware.h"
#endif
#endif

#ifdef INSTRUCTOR_REMOTE
#ifdef STUDENT_JOYSTICK
#error "Please only #define one of INSTRUCTOR_REMOTE or WIRELESS_JOYSTICK in remote_hardware.h"
#endif
#endif

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
