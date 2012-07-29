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

#define XOFFSET 118
#define YOFFSET 116

// Shear mapping: x' = x + m * y
static inline uint8_t shearMapX(uint8_t x, uint8_t y) {
	return ((int16_t)x - XOFFSET - ((int16_t)y - YOFFSET) / 4) + XOFFSET;
	//return ((double)x - XOFFSET - 0.2 * ((double)y - YOFFSET)) + XOFFSET;
}

// Shear mapping: y' = y + m * x
static inline uint8_t shearMapY(uint8_t x, uint8_t y) {
	return ((int16_t)y - YOFFSET - ((int16_t)x - XOFFSET) / 4) + YOFFSET;
	//return ((double)y - YOFFSET - 0.2 * ((double)x - XOFFSET)) + YOFFSET;
}

static void sendData(void)
{
	NORDIC_PACKET testPacket;
	uint8_t x = GetADC5(); // Direction
	uint8_t y = GetADC6(); // Speed
	memset(&testPacket, 0, sizeof(testPacket));

	testPacket.data.array[0] = GetButton() | GetJoyState();
	testPacket.data.array[1] = shearMapX(x, y);
	testPacket.data.array[2] = shearMapY(x, y);

	nordic_TransmitData(&testPacket);
}

int main(void)
{
	//enable interrupts
	sei();

	initHardware();
	nordic_Initialize(0);

	while(1) {
		tglLED();
		sendData();
	}

	return 0;
}
