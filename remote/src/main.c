/*
 * main.c
 *
 *  Created on: Apr 11, 2011
 *      Author: grant
 */
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "remote_hardware.h"
#include "nordic_driver.h"
#include "nordic_hardware_specific.h"

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define XOFFSET 0
#define YOFFSET 0

// Shear mapping: x' = x + m * y
static inline uint8_t shearMapX(uint8_t x, uint8_t y) {
	return ((int16_t)x - XOFFSET - ((int16_t)y - YOFFSET) / 4) + XOFFSET;
}

// Shear mapping: y' = y + m * x
static inline uint8_t shearMapY(uint8_t x, uint8_t y) {
	return ((int16_t)y - YOFFSET - ((int16_t)x - XOFFSET) / 4) + YOFFSET;
}

static void sendData(void)
{
	NORDIC_PACKET testPacket;
	int8_t x = getADC5(); // Direction
	int8_t y = getADC6(); // Speed
	memset(&testPacket, 0, sizeof(testPacket));

#ifdef STUDENT_JOYSTICK
	testPacket.data.array[0] = getBuddyButtons();
	if (isJoystickEnabled()) {
		testPacket.data.array[1] = x - 118; // shearMapX(x, y);
		testPacket.data.array[2] = y - 117; // shearMapY(x, y);
	} else {
		testPacket.data.array[1] = 0;
		testPacket.data.array[2] = 0;
	}
#else //INSTRUCTOR_REMOTE
	testPacket.data.array[0] = getEStop();
	if (x == 0 || x == 1) {
		x = 127;
	} else {
		x = -(x - 129);
	}
	y = y - 128;
	testPacket.data.array[1] = x;
	testPacket.data.array[2] = y;
#endif

	nordic_TransmitData(&testPacket);
}

int main(void)
{
	//enable interrupts
	sei();

	initHardware();
	nordic_Initialize();

	while(1) {
		sendData();
		_delay_ms(5);
	}

	return 0;
}
