/*
 * test.c
 *
 * Created: 6/8/2012 10:22:04 AM
 *  Author: Stew
 */ 

#include <stdio.h>
#include <string.h>
#include "test.h"
#include "nordic_driver.h"
#include "../atmel/wdt_driver.h"
#include "PWCT_io.h"
#include "linear_actuator.h"
#include "bumper.h"
#include "motor_driver.h"

void getStateStr(states state, char *str)
{
	switch(state) {
		case IDLE:
		strcpy(str, "IDLE");
		break;
		case MOVE:
		strcpy(str, "MOVE");
		break;
		case LOAD:
		strcpy(str, "LOAD");
		break;
		default:
		strcpy(str, "ERR ");
		break;
	}
}

static void byte_to_binary(char * b, int x)
{
	b[0] = '\0';

	uint8_t z;
	for (z = 0x80; z > 0; z >>= 1)
	{
		strcat(b, ((x & z) == z) ? "1" : "0");
	}
}

// TEST WIRELESS COMMUNICATION
void testNordicWireless(void)
{
	uint8_t i;
	NORDIC_PACKET packet;
	while(1) {
		WDT_Reset();
		//check for received data

		if(nordic_GetNewPacket(&packet) != 0) {
			char b[9];
			byte_to_binary(b, packet.data.array[0]);
			printf("RX 0b%s", b);
			//printf("RX 0x%02X", packet.data.array[0]);
			for(i = 1; i < sizeof(packet.data.array); i++) {
				printf(", %3d", packet.data.array[i]);
			}
			printf("\n");
			//PORTK.OUTSET = PIN5_bm;
			//_delay_ms(20);
			//PORTK.OUTCLR = PIN5_bm;
			//_delay_ms(20);
		}
	}
}

// test inputs
void testInputs(void)
{
	uint8_t switchState[32];
	printf("|      Remote          |                Panel                      |Limit|             Bumpers              |\r\n");
	printf("| Joystck | LA  | Estp | Buddy Btn | Joystick  | Estp | LA  | OvrRd|     | 1    2    3    4    5    6    7  |\r\n");
	printf("| U D L R | U D |      | U D L R S | U D L R S |      | U D |      |     |                                  |\r\n");
	while(1) {
		SampleInputs();
		GetInputStates(switchState);
		printf("| %1d %1d %1d %1d | %1d %1d |  %1d   | %1d %1d %1d %1d %1d | %1d %1d %1d %1d %1d |  %1d   | %1d %1d |  %1d   |  %1d  |%4d %4d %4d %4d %4d %4d %4d|\r",
				switchState[0]?1:0, switchState[1]?1:0, switchState[2]?1:0, switchState[3]?1:0,
				switchState[4]?1:0, switchState[5]?1:0, switchState[6]?1:0, switchState[7]?1:0,
				switchState[8]?1:0, switchState[9]?1:0, switchState[10]?1:0, switchState[11]?1:0,
				switchState[12]?1:0, switchState[13]?1:0, switchState[14]?1:0, switchState[15]?1:0,
				switchState[16]?1:0, switchState[17]?1:0, switchState[18]?1:0, switchState[19]?1:0,
				switchState[20]?1:0, switchState[21]?1:0, switchState[22], switchState[23],
				switchState[24], switchState[25], switchState[26], switchState[27],
				switchState[28]);
		_delay_ms(250);
	}
}

// Test the motor driver
void testMotorDriver(void)
{
	sendMotorCommand(0, 0);
	sendMotorCommand(4, 0);
	while (1)
	{
		WDT_Reset();

		int i;
		for (i = 0; i < 20; i++)
		{
			sendMotorCommand(0, 60);
			_delay_ms(100);
			WDT_Reset();
		}
		for (i = 0; i < 20; i++)
		{
			sendMotorCommand(0, 0);
			_delay_ms(100);
			WDT_Reset();
		}

		for (i = 0; i <= 60; i++)
		{
			WDT_Reset();
			sendMotorCommand(0, i);
			sendMotorCommand(4, i);
			_delay_ms(20);
		}

		WDT_Reset();
		_delay_ms(100);

		for (; i >= 0; i--)
		{
			WDT_Reset();
			sendMotorCommand(0, i);
			sendMotorCommand(4, i);
			_delay_ms(20);
		}

		WDT_Reset();
		_delay_ms(450);
		WDT_Reset();
		_delay_ms(450);
		WDT_Reset();
		_delay_ms(450);
		WDT_Reset();
		_delay_ms(450);
	}
}

// Warning: possibly obsolete, untested test
// hard wire controls
void hardWireControls(void)
{
	uint8_t prtj = 0;
	while(1) {
		prtj = PORTJ.IN;

		switch(PORTK.IN & (PIN0_bm | PIN3_bm)) {
		case 0x01:
			RaisePlatform();
			break;
		case 0x08:
			LowerPlatform();
			break;
		default:
			StopPlatform();
			break;
		}

		if((prtj & PIN3_bm) == 0) {	//forward
			PORTH.OUTSET = PIN1_bm;
		} else {
			PORTH.OUTCLR = PIN1_bm;
		}
		if((prtj & PIN4_bm) == 0) {	//reverse
			PORTH.OUTSET = PIN0_bm;
		} else {
			PORTH.OUTCLR = PIN0_bm;
		}
		if((prtj & PIN5_bm) == 0) {	//left
			PORTH.OUTSET = PIN4_bm;
		} else {
			PORTH.OUTCLR = PIN4_bm;
		}
		if((prtj & PIN6_bm) == 0) {	//right
			PORTH.OUTSET = PIN3_bm;
		} else {
			PORTH.OUTCLR = PIN3_bm;
		}
		if((prtj & PIN7_bm) == 0) {	//select
			PORTH.OUTSET = PIN5_bm;
		} else {
			PORTH.OUTCLR = PIN5_bm;
		}
	}
}

// Warning: definitely obsolete
// Test bumpers
void testBumpers(void)
{
	uint16_t bumpers[16];
	while(1) {
		WDT_Reset();
		GetBumperValues(bumpers);
		printf("%4d,%4d,%4d,%4d,%4d,%4d,%4d\n",
			bumpers[0],
			bumpers[1], bumpers[2], bumpers[3],
			bumpers[4], bumpers[5], bumpers[6]
			);
		BumperAlgorithm();
		_delay_ms(100);
	}
}
