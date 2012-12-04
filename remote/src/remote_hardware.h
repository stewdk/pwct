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
#error "Please #define either INSTRUCTOR_REMOTE or STUDENT_JOYSTICK in remote_hardware.h"
#endif
#endif

#ifdef INSTRUCTOR_REMOTE
#ifdef STUDENT_JOYSTICK
#error "Please only #define one of INSTRUCTOR_REMOTE or STUDENT_JOYSTICK in remote_hardware.h"
#endif
#endif

typedef struct {
	volatile uint8_t *pin;
	uint8_t pin_bm;
	volatile uint8_t previous_values;
	volatile uint8_t debounced_value;
} debounced_input;

uint8_t getADC5(void);
uint8_t getADC6(void);
#ifdef INSTRUCTOR_REMOTE
uint8_t getEStop(void);
#endif // INSTRUCTOR_REMOTE
void initHardware(void);
void setLED(void);
void clrLED(void);
void tglLED(void);
void setLEDDelay();
void clrLEDDelay();
#ifdef STUDENT_JOYSTICK
uint8_t isJoystickEnabled();
uint8_t getBuddyButtons();
#endif // STUDENT_JOYSTICK


#endif /* REMOTE_HARDWARE_H_ */
