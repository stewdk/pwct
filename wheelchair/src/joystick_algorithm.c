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

// Input to time-based filters (acceleration, tremor dampening)
static volatile int16_t gSpeedPreFilter = 0;
static volatile int16_t gDirPreFilter = 0;

static volatile double gSpeedBetweenFilters = 0;
static volatile double gDirBetweenFilters = 0;

// Output from time-based filters
static volatile int16_t gSpeedPostFilter = 0;
static volatile int16_t gDirPostFilter = 0;

static volatile uint8_t gIsOuterDeadBand = 0;
static volatile uint8_t gOuterDeadBandTime = 0;

void joystickAlgorithmInit()
{
	// TCD1 is the timer acceleration and tremor filtering
	TCD1.CTRLA = TC_CLKSEL_DIV1_gc; // 1 tick = 31.25 nanoseconds
	TCD1.CTRLB = TC_WGMODE_FRQ_gc;
	TCD1.INTCTRLB = TC_CCAINTLVL_MED_gc;
	TCD1.CCA = 32000; // Goal: interrupt every 1 millisecond

	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
}

static uint8_t isOuterDeadBand(int16_t speed, int16_t dir)
{
	if (speed < 0) {
		speed = -speed;
	}
	if (dir < 0) {
		dir = -dir;
	}
	if (menuGetOuterDeadBand() && speed + dir > 60) {
		return 1;
	} else {
		return 0;
	}
}

static uint8_t isOuterDeadBandTimeout()
{
	uint8_t isTimeout = 0;
	uint8_t timeoutTime = menuGetOuterDeadBand() - 1;
	if (timeoutTime >= 0) {
		AVR_ENTER_CRITICAL_REGION();
		if (gOuterDeadBandTime >= timeoutTime) {
			isTimeout = 1;
		}
		AVR_LEAVE_CRITICAL_REGION();
	}
	return isTimeout;
}

static int16_t centerDeadBand(int16_t input, uint8_t deadBand)
{
	if (input > 0) {
		input -= deadBand;
		if (input < 0) {
			input = 0;
		}
	}
	if (input < 0) {
		input += deadBand;
		if (input > 0) {
			input = 0;
		}
	}
	return input;
}

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

	gIsOuterDeadBand = isOuterDeadBand(speed, dir);
	if (gIsOuterDeadBand && isOuterDeadBandTimeout()) {
		*returnSpeed = 0;
		*returnDir = 0;
		AVR_ENTER_CRITICAL_REGION();
		gSpeedPreFilter = 0;
		gDirPreFilter = 0;
		gSpeedBetweenFilters = 0;
		gDirBetweenFilters = 0;
		gSpeedPostFilter = 0;
		gDirPostFilter = 0;
		AVR_LEAVE_CRITICAL_REGION();
		return;
	}

	if (menuGetInvert()) {
		speed = -speed;
	}

	// Proportional joystick as switch joystick
	if (menuGetPropAsSwitch())
	{
		// TODO diagonal
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
		speed = centerDeadBand(speed, menuGetCenterDeadBand());
		dir = centerDeadBand(dir, menuGetCenterDeadBand());

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

	// We've applied the direct-mapped logic, now it's time to hand it off to the filters
	AVR_ENTER_CRITICAL_REGION();
	gSpeedPreFilter = speed;
	gDirPreFilter = dir;

	speed = gSpeedPostFilter;
	dir = gDirPostFilter;
	AVR_LEAVE_CRITICAL_REGION();

	// One last post-filter operation
	*returnSpeed = centerDeadBand(speed, 1);
	*returnDir = centerDeadBand(dir, 1);
}

// Filter topography and variable naming:
// preFilter --> [low-pass] --> betweenFilters --> [accel/decel] --> postFilter
ISR(TCD1_CCA_vect)
{
	static uint8_t accelerationCount = 0;
	static uint8_t decelerationCount = 0;
	static uint16_t outerDeadBandMillisecondCount = 0;

	if (gIsOuterDeadBand) {
		outerDeadBandMillisecondCount++;
		if (outerDeadBandMillisecondCount >= 500) {
			outerDeadBandMillisecondCount = 0;
			gOuterDeadBandTime++;
		}
	} else {
		outerDeadBandMillisecondCount = 0;
		gOuterDeadBandTime = 0;
	}

	// Low-pass filter (aka Tremor Dampening aka Tremor Suppression aka Sensitivity)
	gSpeedBetweenFilters = gSpeedBetweenFilters + (double)menuGetSensitivity() * (gSpeedPreFilter - gSpeedBetweenFilters);
	gDirBetweenFilters = gDirBetweenFilters + (double)menuGetSensitivity() * (gDirPreFilter - gDirBetweenFilters);

	// Acceleration/deceleration: must wait X milliseconds before speed/dir is changed by 1
	accelerationCount++;
	decelerationCount++;
	if (accelerationCount >= menuGetAcceleration())
	{
		accelerationCount = 0;

		if (gSpeedPostFilter >= 0 && gSpeedBetweenFilters > gSpeedPostFilter) {
			gSpeedPostFilter++;
		}
		if (gSpeedPostFilter <= 0 && gSpeedBetweenFilters < gSpeedPostFilter) {
			gSpeedPostFilter--;
		}
		if (gDirPostFilter >= 0 && gDirBetweenFilters > gDirPostFilter) {
			gDirPostFilter++;
		}
		if (gDirPostFilter <= 0 && gDirBetweenFilters < gDirPostFilter) {
			gDirPostFilter--;
		}
	}

	if (decelerationCount >= menuGetDeceleration())
	{
		decelerationCount = 0;

		if (gSpeedPostFilter > 0 && gSpeedBetweenFilters < gSpeedPostFilter) {
			gSpeedPostFilter--;
			if (gSpeedBetweenFilters < 0) {
				gSpeedPostFilter--;
			}
		}
		if (gSpeedPostFilter < 0 && gSpeedBetweenFilters > gSpeedPostFilter) {
			gSpeedPostFilter++;
			if (gSpeedBetweenFilters > 0) {
				gSpeedPostFilter++;
			}
		}

		if (gDirPostFilter > 0 && gDirBetweenFilters < gDirPostFilter) {
			gDirPostFilter--;
			if (gDirBetweenFilters < 0) {
				gDirPostFilter--;
			}
		}
		if (gDirPostFilter < 0 && gDirBetweenFilters > gDirPostFilter) {
			gDirPostFilter++;
			if (gDirBetweenFilters > 0) {
				gDirPostFilter++;
			}
		}
	}
}
