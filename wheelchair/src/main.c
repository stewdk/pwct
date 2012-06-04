/* This file has been prepared for Doxygen automatic documentation generation.*/
/*! \file *********************************************************************
 *
 * \brief  PWCT main function source file
 *
 *      This file contains the main function of the Power Wheelchair Trainer
 *
 * \par Target note:
 *      This code is written for an XMEGA 64 A1 device
 *
 * \author
 *      Jeff VanOss, Anderson Peck, Paul Shields
 *
 * $Revision: 1 $
 * $Date: 03-28-2011$  \n
 *
 *****************************************************************************/

#include "../atmel/avr_compiler.h"
#include "../atmel/clksys_driver.h"
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include "util.h"
#include <stdio.h>
#include "nordic_driver.h"
#include "linear_actuator.h"
#include "bumper.h"
#include "PWCT_io.h"
#include <stdlib.h>     // strtol
#include "../atmel/wdt_driver.h"
#define ACTUATOR_THRESHOLD 10000

typedef enum {
	IDLE, MOVE, LOAD
}states;

double PLATFORM_COUNT;
//static char GSTR[64];
const char *byte_to_binary(int x);
void getStateStr(states state, char *str);

/*! \brief Main function
 *
 *  This function initializes the hardware, starts monitoring input signals.
 */
int main( void )
{
	//uint8_t i;
	uint8_t moveDir = 0;
	uint8_t actuatorSwitchState = 0;
	uint8_t limitSwitchPressedFlag = 0;
	//uint8_t prtj = 0;
	states state = IDLE;
	//NORDIC_PACKET packet;
	//uint8_t switchState[32];
	//uint16_t bumpers[16];

	//Setup the 32MHz Clock
	//start 32MHz oscillator
	CLKSYS_Enable( OSC_RC32MEN_bm );
	//wait for 32MHz oscillator to stabilize
	while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	//set clock as internal 32MHz RC oscillator
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );

	WDT_EnableAndSetTimeout(WDT_PER_512CLK_gc);	//set watchdog timer for 0.5s period

	dbgLEDinit();
	dbgUSARTinit();

	initLinearActuators();
	PLATFORM_COUNT = 0;
//	initBumpers();
	initPWCTio();

	/* Enable all three levels global interrupts. */
	sei();

//	printf("\n\rReset\n\r");

	nordic_Initialize(1);
/*
	//TEST3 WIRELESS COMMUNICATION
	while(1) {
		//check for received data

		if(nordic_GetNewPacket(&packet) != 0) {

			printf("RX 0b%s", byte_to_binary(packet.data.array[0]));
			//printf("RX 0x%02X", packet.data.array[0]);
			for(i = 1; i < sizeof(packet.data.array); i++) {
				printf(", %3d", packet.data.array[i]);
			}
			printf("\n");
			PORTK.OUTSET = PIN5_bm;
			_delay_ms(20);
			PORTK.OUTCLR = PIN5_bm;
			_delay_ms(20);
		}
	}
*/
/*
	//test inputs
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
*/
/*
	//hard wire controls
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
*/
/*
	//Test bumpers
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
*/

	//Run Operational State Machine
	while(1) {
		WDT_Reset();
//		dbgLEDtgl();

//		BumperAlgorithm();

		//check inputs for state changes
		SampleInputs();
		moveDir = GetMoveDirection();

		actuatorSwitchState = ActuatorSwitchPressed();
		limitSwitchPressedFlag = LimitSwitchPressed();
		if (PLATFORM_COUNT >= ACTUATOR_THRESHOLD){
			//limitSwitchPressedFlag = 1; //the platform is all the way up
		}
		if (PLATFORM_COUNT < ACTUATOR_THRESHOLD){
			//limitSwitchPressedFlag = 0; //the platform is not all the way up, could be anywhere below top
		}
//		PrintBumperStates();
//		PrintLACurrents();
//		PrintRightCornerBumper();
//		PrintLeftCornerBumper();
//		PrintRightFrontBumper();
		if(EmergencyStopPressed()) {
			state = IDLE;
		}
		else if(!limitSwitchPressedFlag) {
			state = LOAD;
		}
		else if( (state == IDLE || state == LOAD) && actuatorSwitchState) {
			state = LOAD;
		}
		else if(state == LOAD && limitSwitchPressedFlag) {
			state = IDLE;
		}
		else if(moveDir != 0) {
			state = MOVE;
		}
		else {
			state = IDLE;
		}

//		getStateStr(state, GSTR);
//		printf("State:%s\tActuator:%2d\tLimit:%2d\tMove:%2d\r", GSTR, actuatorSwitchState, limitSwitchPressedFlag, moveDir);

		//set state output
		switch(state) {
		case IDLE:
//			dbgLEDset();
			StopPlatform();
			StopMove();
			//turn off platform down LED
			PORTK.OUTCLR = PIN5_bm;
			break;
		case LOAD:
//			dbgLEDset();
			StopMove();
			switch(actuatorSwitchState) {
			case 0:	//actuator switch not pressed, stop platform
				StopPlatform();
				break;
			case 1:	//actuator switch down, lower platform
				LowerPlatform();
				if (PLATFORM_COUNT > 0){
					PLATFORM_COUNT -= 0.5;
				}
				break;
			case 2: // actuator switch up, raise platform
				RaisePlatform();
				if (PLATFORM_COUNT < ACTUATOR_THRESHOLD){
					PLATFORM_COUNT += 0.5;
				}
				break;
			}
			//turn on platform down LED
			PORTK.OUTSET = PIN5_bm;
			break;
		case MOVE:
//			dbgLEDclr();
			StopPlatform();
			Move(moveDir);
			//turn off platform down LED
			PORTK.OUTCLR = PIN5_bm;
			break;
		}
//		_delay_ms(20);
	}
	return 1;
}

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

const char *byte_to_binary(int x)
{
    static char b[9];
    b[0] = '\0';

    uint8_t z;
    for (z = 0x80; z > 0; z >>= 1)
    {
        strcat(b, ((x & z) == z) ? "1" : "0");
    }
    return b;
}

