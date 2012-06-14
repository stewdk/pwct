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

#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>     // strtol
#include "../atmel/avr_compiler.h"
#include "../atmel/clksys_driver.h"
#include "util.h"
#include "nordic_driver.h"
#include "linear_actuator.h"
#include "PWCT_io.h"
#include "../atmel/wdt_driver.h"
#include "motor_driver.h"
#include "lcd_driver.h"
//#include "test.h"

//static char GSTR[64];

static void pwctIOLogic(void)
{
	static uint8_t estopDebounceFlg = 0;
	static uint8_t EstopPulseSent = 0;
	if(!EmergencyStopPressed()) { // Not pressed
		estopDebounceFlg = 0;
		EstopPulseSent = 0;
	}
	else if(!EstopPulseSent) { // Pressed
		estopDebounceFlg++;
		if(estopDebounceFlg >= 3) {
			PulsePGDTEstop();
			EstopPulseSent = 1;
		}
	}

	//set bumper override led
	if(getPANEL_BUMPER_OVERRIDE()) {
		PORTK.OUTCLR = PIN4_bm;
	} else {
		PORTK.OUTSET = PIN4_bm;
	}

	//check if panel bumper override was toggled
	if(!getPANEL_BUMPER_OVERRIDE()) {
		//ResetBumperThreshold();
	}
}

/*! \brief Main function
 *
 *  This function initializes the hardware, starts monitoring input signals.
 */
int main( void )
{
	uint8_t moveDir = 0;
	uint8_t actuatorSwitchState = 0;
	uint8_t limitSwitchPressedFlag = 0;
	states state = IDLE;

	//Setup the 32MHz Clock
	//start 32MHz oscillator
	CLKSYS_Enable( OSC_RC32MEN_bm );
	//wait for 32MHz oscillator to stabilize
	while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	//set clock as internal 32MHz RC oscillator
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );

	// Enable global interrupts.
	sei();
	initMotorDriver();

	dbgLEDinit();
	dbgUSARTinit();

	initLinearActuators();

	initPWCTio();

	nordic_Initialize(1);

	initLCDDriver();

	WDT_EnableAndSetTimeout(WDT_PER_512CLK_gc);	//set watchdog timer for 0.5s period

	printf("\nReset\n");

	//testPropJoy();

	//testNordicWireless();

	//testInputs();

	//testMotorDriver();

	//Run Operational State Machine
	while(1) {
		WDT_Reset();
//		dbgLEDtgl();

		//check inputs for state changes
		SampleInputs();
		pwctIOLogic();
		moveDir = GetMoveDirection();

		actuatorSwitchState = ActuatorSwitchPressed();
		limitSwitchPressedFlag = LimitSwitchPressed();

//		PrintLACurrents();

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
				break;
			case 2: // actuator switch up, raise platform
				RaisePlatform();
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
