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

#define XOFFSET 118
#define YOFFSET 116

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
	uint8_t x = GetADC5(); // Direction
	uint8_t y = GetADC6(); // Speed
	memset(&testPacket, 0, sizeof(testPacket));

#ifdef STUDENT_JOYSTICK
	testPacket.data.array[0] = getBuddyButtons();
	if (isJoystickEnabled()) {
		testPacket.data.array[1] = x; // shearMapX(x, y);
		testPacket.data.array[2] = y; // shearMapY(x, y);
	} else {
		testPacket.data.array[1] = 0;
		testPacket.data.array[2] = 0;
	}
#else //INSTRUCTOR_REMOTE
	testPacket.data.array[0] = GetButton() | GetJoyState();
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
	nordic_Initialize(0);


#ifdef STUDENT_JOYSTICK

	while(1) {
		sendData();
		_delay_ms(5);
	}

#else // INSTRUCTOR_REMOTE

	sbi(MCUCR, SM1);	//Power Down sleep mode

	while(1) {
		//go to sleep
		nordic_PowerDown();
		clrLED();

		sbi(MCUCR, SE);	//sleep enabled
		asm volatile ("sleep");	//Go to sleep
		cbi(MCUCR, SE);	//sleep disabled

		nordic_PowerUp();

		//set last button pressed time
		resetTimeOut();

		while(1) {
			tglLED();

			if(isTimeOut()) {
				resetTimeOut();
				//if no buttons are pressed quit out and go to sleep
				if(!GetButton() && !GetJoyState()) {
					break;
				}
				else {
					//send out latest buttons
					//this way if wheelchair doesn't get a packet every timeout period
					//it can assume the remote is out of range
					sendData();
				}
			}

			//send only if button was changed
			else if(hasButtonChanged()) {
				clrButtonChanged();
				sendData();
				resetTimeOut();
			}
		}
	}
#endif

	return 0;
}
