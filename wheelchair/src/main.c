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
//#include "test.h"

static void getProportionalMoveDirection(int16_t *speed, int16_t *dir)
{
	*speed = getWirelessPropJoySpeed();
	*dir = getWirelessPropJoyDirection();

	// fwd/rev offset
	if (*speed) {
		*speed -= 116;
	}

	// right/left offset
	if (*dir) {
		*dir -= 118;
	}

	// Proportional joystick as switch joystick
	if (menuGetPropAsSwitch())
	{
		uint8_t threshold = 50;
		if (*speed > threshold) {
			*speed = menuGetTopFwdSpeed();
		} else if (*speed < -threshold) {
			*speed = -menuGetTopRevSpeed();
		} else {
			*speed = 0;
		}

		if (*dir > threshold) {
			*dir = menuGetTopTurnSpeed();
		} else if (*dir < -threshold) {
			*dir = -menuGetTopTurnSpeed();
		} else {
			*dir = 0;
		}
	}
	else
	{
		// center dead band
		// Todo: come up with a better algorithm
		if (*speed > 0) {
			*speed -= menuGetCenterDeadBand();
			if (*speed < 0) {
				*speed = 0;
			}
		}
		if (*speed < 0) {
			*speed += menuGetCenterDeadBand();
			if (*speed > 0) {
				*speed = 0;
			}
		}
		if (*dir > 0) {
			*dir -= menuGetCenterDeadBand();
			if (*dir < 0) {
				*dir = 0;
			}
		}
		if (*dir < 0) {
			*dir += menuGetCenterDeadBand();
			if (*dir > 0) {
				*dir = 0;
			}
		}

		// fwd/rev throw
		if (*speed > 0) {
			*speed *= menuGetFwdThrow();
		}
		if (*speed < 0) {
			*speed *= menuGetRevThrow();
		}
		// turn throw
		*dir *= menuGetTurnThrow();

		// Top speeds
		if (*speed > menuGetTopFwdSpeed()) {
			// max forward speed
			*speed = menuGetTopFwdSpeed();
		} else if (*speed < -menuGetTopRevSpeed()) {
			// max reverse speed
			*speed = -menuGetTopRevSpeed();
		}

		// max turn speed
		if (*dir > menuGetTopTurnSpeed()) {
			*dir = menuGetTopTurnSpeed();
		} else if (*dir < -menuGetTopTurnSpeed()) {
			*dir = -menuGetTopTurnSpeed();
		}
	}
}

static void setMotors(int16_t speed, int16_t dir)
{
	if (speed >= 0) {
		sendMotorCommand(MOTOR_CMD_DRIVE_FORWARD_MIXED_MODE, speed);
	} else {
		speed = -speed;
		sendMotorCommand(MOTOR_CMD_DRIVE_BACKWARDS_MIXED_MODE, speed);
	}
	if (dir >= 0) {
		sendMotorCommand(MOTOR_CMD_TURN_RIGHT_MIXED_MODE, dir);
	} else {
		dir = -dir;
		sendMotorCommand(MOTOR_CMD_TURN_LEFT_MIXED_MODE, dir);
	}
}

static void eStop(void)
{
	motorEStop();
	lcdText("E-stop", "", 0);
	while (1)
	{
		WDT_Reset();
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
	initMotorDriver();

	dbgLEDinit();
	dbgUSARTinit();

	initLinearActuators();

	initPWCTio();

	nordic_Initialize(1);

	initLCDDriver();

	WDT_EnableAndSetTimeout(WDT_PER_128CLK_gc);	//set watchdog timer for 0.125s period

	printf("\nReset\n");

	//testJoystickDriveMotors();

	//testPropJoy();

	//testNordicWireless();

	//testInputs();

	//testMotorDriver();

	lcdText("PWCT  Build Date", "     " __DATE__, 0);

	//Run Operational State Machine
	while(1) {
		WDT_Reset();

		getProportionalMoveDirection(&speed, &dir);
		menuUpdate(speed, dir);

		//check inputs for state changes
		SampleInputs();
		moveDir = getSwitchMoveDirection();

		actuatorSwitchState = ActuatorSwitchPressed();

		if (PanelEStopPressed() || getInstructorEStop()) {
			eStop();
		} else if (!LimitSwitchPressed() || ((state == IDLE || state == LOAD) && actuatorSwitchState)) {
			state = LOAD;
		} else if (LimitSwitchPressed() && (moveDir != 0 || speed != 0 || dir != 0)) {
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
				break;
			case 2: // actuator switch up, raise platform
				RaisePlatform();
				break;
			}
			//turn on platform down LED
			PORTK.OUTSET = PIN5_bm;
			break;
		case MOVE:
			StopPlatform();
			OmniMove(moveDir);
			setMotors(speed, dir);
			//turn off platform down LED
			PORTK.OUTCLR = PIN5_bm;
			break;
		}
		//previousState = state;
	}
	return 1;
}
