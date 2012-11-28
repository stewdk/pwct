/* This file has been prepared for Doxygen automatic documentation generation... kind of.*/
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
 *      Stew Hildebrand, Jeff VanOss, Anderson Peck, Paul Shields
 *
 * $Revision: 1 $
 * $Date: 03-28-2011$  \n
 *
 *****************************************************************************/

#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>     // strtol
#include "../atmel/clksys_driver.h"
#include "../atmel/wdt_driver.h"
#include "util.h"
#include "nordic_driver.h"
#include "linear_actuator.h"
#include "PWCT_io.h"
#include "motor_driver.h"
#include "lcd_driver.h"
#include "menu.h"
#include "joystick_algorithm.h"
//#include "test.h"

static void eStop(const char *estopText)
{
	motorEStop();
	while (1)
	{
		lcdText(estopText, "Ver. " __DATE__, 0);
		WDT_Reset();
	}
}

static void displayResetReason(double delayTime_ms)
{
	uint8_t status = RST.STATUS;
	char lcdLine[LCD_NUM_CHARACTERS+1];
	lcdLine[LCD_NUM_CHARACTERS] = '\0';
	sprintf(lcdLine, "RST.STATUS=0x%02x", status);

	RST.STATUS = status & 0x3F;

	if (status & RST_SDRF_bm) {
		lcdText("Spike Detect Rst", lcdLine, 1);
		while (1) {
		}
	}
	if (status & RST_SRF_bm) {
		lcdText("Software Reset", lcdLine, 1);
		while (1) {
		}
	}
	if (status & RST_PDIRF_bm) {
		lcdText("PDI Reset", lcdLine, 1);
		_delay_ms(delayTime_ms);
	}
	if (status & RST_WDRF_bm) {
		lcdText("Watchdog Reset", lcdLine, 1);
		while (1) {
		}
	}
	if (status & RST_BORF_bm) {
		lcdText("Brown-out Reset", lcdLine, 1);
		while (1) {
		}
	}
	if (status & RST_EXTRF_bm) {
		lcdText("External Reset", lcdLine, 1);
		_delay_ms(delayTime_ms);
	}
	if (status & RST_PORF_bm) {
		lcdText("Power-on Reset", lcdLine, 1);
		_delay_ms(delayTime_ms);
	}
	if (status == 0) {
		lcdText("RST.STATUS == 0", lcdLine, 1);
		while (1) {
		}
	}
}

/*! \brief Main function
 *
 *  This function initializes the hardware, starts monitoring input signals.
 */
int main( void )
{
	uint8_t actuatorSwitchState = 0;
	states state = IDLE;
	//states previousState = IDLE;
	int16_t speed;
	int16_t dir;

	//Setup the 32MHz Clock
	//start 32MHz oscillator
	CLKSYS_Enable( OSC_RC32MEN_bm );
	//wait for 32MHz oscillator to stabilize
	while ( CLKSYS_IsReady( OSC_RC32MRDY_bm ) == 0 );
	//set clock as internal 32MHz RC oscillator
	CLKSYS_Main_ClockSource_Select( CLK_SCLKSEL_RC32M_gc );

	// Enable global interrupts.
	sei();
	joystickAlgorithmInit();
	initMotorDriver();

	dbgLEDinit();
	dbgUSARTinit();

	initLinearActuators();

	initPWCTio();

	nordic_Initialize();

	initLCDDriver();

	displayResetReason(500);

	menuInit();

	WDT_EnableAndSetTimeout(WDT_PER_128CLK_gc);	//set watchdog timer for 0.125s period

	printf("\nReset\n");

	//testJoystickDriveMotors();

	//testPropJoy();

	//testNordicWireless();

	//testInputs();

	//testMotorDriver();

	//Run Operational State Machine
	while(1) {
		WDT_Reset();

		getProportionalMoveDirection(&speed, &dir);
		menuUpdate(speed, dir);

		//check inputs for state changes
		SampleInputs();

		actuatorSwitchState = ActuatorSwitchPressed();

		if (nordic_getInstructorEStop()) {
			eStop("Remote E-stop");
		} else if (PanelEStopPressed()) {
			eStop("Panel E-stop");
		} else if (menuGetIsPlatformDown() || ((state == IDLE || state == LOAD) && actuatorSwitchState)) {
			state = LOAD;
		} else if (!menuGetIsPlatformDown() && (speed != 0 || dir != 0)) {
			state = MOVE;
		} else {
			state = IDLE;
		}

		/*
		if (previousState != state)
		{
			switch (state) {
			case IDLE:
				printf("Idle\n");
				break;
			case MOVE:
				printf("Move\n");
				break;
			case LOAD:
				printf("Load\n");
				break;
			}
			previousState = state;
		}
		*/

		//set state output
		switch(state) {
		case IDLE:
			StopPlatform();
			OmniStopMove();
			//turn off platform down LED
			PORTK.OUTCLR = PIN5_bm;
			break;
		case LOAD:
			OmniStopMove();
			switch(actuatorSwitchState) {
			case 0:	//actuator switch not pressed, stop platform
				StopPlatform();
				break;
			case 1:	//actuator switch down, lower platform
				LowerPlatform();
				menuPlatformDownPushed();
				break;
			case 2: // actuator switch up, raise platform
				RaisePlatform();
				menuPlatformUpPushed();
				break;
			}
			//turn on platform down LED
			PORTK.OUTSET = PIN5_bm;
			break;
		case MOVE:
			StopPlatform();
			setMotors(speed, dir);
			//turn off platform down LED
			PORTK.OUTCLR = PIN5_bm;
			break;
		}
	}
	return 1;
}
