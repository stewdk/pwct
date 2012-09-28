/*
 * joystick_algorithm.c
 *
 * Created: 9/27/2012 9:52:10 PM
 *  Author: Stew
 */ 

#include <stdint.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include "../atmel/avr_compiler.h"
#include "nordic_driver.h"
#include "menu.h"
//#include "util.h"

void joystickAlgorithmInit()
{
	// TCD1 is the timer acceleration and tremor filtering
	TCD1.CTRLA = TC_CLKSEL_DIV1_gc; // 1 tick = 31.25 nanoseconds
	TCD1.CTRLB = TC_WGMODE_FRQ_gc;
	TCD1.INTCTRLB = TC_CCAINTLVL_MED_gc;
	TCD1.CCA = 32000; // Goal: interrupt every 1 millisecond

	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
}

// Input to time-based filters (acceleration, tremor dampening)
volatile int16_t gSpeedPreFilter = 0;
volatile int16_t gDirPreFilter = 0;

// Output from time-based filters
volatile int16_t gSpeedPostFilter = 0;
volatile int16_t gDirPostFilter = 0;

void getProportionalMoveDirection(int16_t *returnSpeed, int16_t *returnDir)
{
	int16_t speed = nordic_getWirelessPropJoySpeed();
	int16_t dir = nordic_getWirelessPropJoyDirection();

	// fwd/rev offset
	if (speed) {
		speed -= 116;
	}

	// right/left offset
	if (dir) {
		dir -= 118;
	}

	if (menuGetInvert()) {
		speed = -speed;
	}

	// Proportional joystick as switch joystick
	if (menuGetPropAsSwitch())
	{
		// TODO lower thresholds?
		uint8_t threshold = 50;
		if (speed > threshold) {
			speed = menuGetTopFwdSpeed();
		} else if (speed < -threshold) {
			speed = -menuGetTopRevSpeed();
		} else {
			speed = 0;
		}

		if (dir > threshold) {
			dir = menuGetTopTurnSpeed();
		} else if (dir < -threshold) {
			dir = -menuGetTopTurnSpeed();
		} else {
			dir = 0;
		}
	}
	else
	{
		// center dead band
		// Todo: come up with a better algorithm... or not
		if (speed > 0) {
			speed -= menuGetCenterDeadBand();
			if (speed < 0) {
				speed = 0;
			}
		}
		if (speed < 0) {
			speed += menuGetCenterDeadBand();
			if (speed > 0) {
				speed = 0;
			}
		}
		if (dir > 0) {
			dir -= menuGetCenterDeadBand();
			if (dir < 0) {
				dir = 0;
			}
		}
		if (dir < 0) {
			dir += menuGetCenterDeadBand();
			if (dir > 0) {
				dir = 0;
			}
		}

		// fwd/rev throw
		if (speed > 0) {
			speed *= menuGetFwdThrow();
		}
		if (speed < 0) {
			speed *= menuGetRevThrow();
		}
		// turn throw
		dir *= menuGetTurnThrow();

		// Top speeds
		if (speed > menuGetTopFwdSpeed()) {
			// max forward speed
			speed = menuGetTopFwdSpeed();
		} else if (speed < -menuGetTopRevSpeed()) {
			// max reverse speed
			speed = -menuGetTopRevSpeed();
		}

		// max turn speed
		if (dir > menuGetTopTurnSpeed()) {
			dir = menuGetTopTurnSpeed();
		} else if (dir < -menuGetTopTurnSpeed()) {
			dir = -menuGetTopTurnSpeed();
		}
	}

	AVR_ENTER_CRITICAL_REGION();
	gSpeedPreFilter = speed;
	gDirPreFilter = dir;

	*returnSpeed = gSpeedPostFilter;
	*returnDir = gDirPostFilter;
	AVR_LEAVE_CRITICAL_REGION();
}

ISR(TCD1_CCA_vect)
{
	// Acceleration/deceleration: must wait X milliseconds before speed/dir is changed by 1
	static uint8_t accelerationCount = 0;
	static uint8_t decelerationCount = 0;
	accelerationCount++;
	decelerationCount++;
	if (accelerationCount >= menuGetAcceleration())
	{
		accelerationCount = 0;

		if (gSpeedPostFilter >= 0 && gSpeedPreFilter > gSpeedPostFilter) {
			gSpeedPostFilter++;
		}
		if (gSpeedPostFilter <= 0 && gSpeedPreFilter < gSpeedPostFilter) {
			gSpeedPostFilter--;
		}
		if (gDirPostFilter >= 0 && gDirPreFilter > gDirPostFilter) {
			gDirPostFilter++;
		}
		if (gDirPostFilter <= 0 && gDirPreFilter < gDirPostFilter) {
			gDirPostFilter--;
		}
	}

	if (decelerationCount >= menuGetDeceleration())
	{
		decelerationCount = 0;

		if (gSpeedPostFilter > 0 && gSpeedPreFilter < gSpeedPostFilter) {
			gSpeedPostFilter--;
			if (gSpeedPreFilter < 0) {
				gSpeedPostFilter--;
			}
		}
		if (gSpeedPostFilter < 0 && gSpeedPreFilter > gSpeedPostFilter) {
			gSpeedPostFilter++;
			if (gSpeedPreFilter > 0) {
				gSpeedPostFilter++;
			}
		}

		if (gDirPostFilter > 0 && gDirPreFilter < gDirPostFilter) {
			gDirPostFilter--;
			if (gDirPreFilter < 0) {
				gDirPostFilter--;
			}
		}
		if (gDirPostFilter < 0 && gDirPreFilter > gDirPostFilter) {
			gDirPostFilter++;
			if (gDirPreFilter > 0) {
				gDirPostFilter++;
			}
		}
	}
}
