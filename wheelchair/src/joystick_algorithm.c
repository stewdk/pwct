/*
 * joystick_algorithm.c
 *
 * Created: 9/27/2012 9:52:10 PM
 *  Author: Stew
 */ 

#include <stdint.h>
#include <math.h>
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
static volatile uint8_t gIsOuterDeadBandTimeout = 0;

static volatile uint8_t gOverridden = 0;

void joystickAlgorithmInit()
{
	// TCD1 is the timer acceleration and tremor filtering
	TCD1.CTRLA = TC_CLKSEL_DIV1_gc; // 1 tick = 31.25 nanoseconds
	TCD1.CTRLB = TC_WGMODE_FRQ_gc;
	TCD1.INTCTRLB = TC_CCAINTLVL_MED_gc;
	TCD1.CCA = 32000; // Goal: interrupt every 1 millisecond

	PMIC.CTRL |= PMIC_MEDLVLEN_bm;
}

static void outerDeadBandLogic(int16_t speed, int16_t dir)
{
	// Calculate joystick distance from center using pythagorean theorem
	double r = sqrt(speed * speed + dir * dir);

	AVR_ENTER_CRITICAL_REGION();
	if (menuGetOuterDeadBand(gOverridden) && (
		(r > 60) ||
		(gIsOuterDeadBand && gIsOuterDeadBandTimeout && (r > menuGetCenterDeadBand(gOverridden) + 5))
		)) {
		gIsOuterDeadBand = 1;
	} else {
		gIsOuterDeadBand = 0;
	}
	AVR_LEAVE_CRITICAL_REGION();
}

static void centerDeadBand(int16_t *x, int16_t *y, uint8_t deadBand)
{
	double theta;
	double r;

	// Convert to polar coordinates
	r = sqrt((*x) * (*x) + (*y) * (*y));
	theta = atan2(*y, *x);

	// Apply the deadband
	r = r - (double)deadBand;
	if (r < 0) {
		r = 0;
	}

	// Convert back to cartesian coordinates
	*x = (r * cos(theta));
	*y = (r * sin(theta));
}

void getProportionalMoveDirection(int16_t *returnSpeed, int16_t *returnDir)
{
	int16_t speed = nordic_getInstructorSpeed();
	int16_t dir = nordic_getInstructorDirection();

	if (speed >= -1 && speed <= 1 &&
		dir   >= -1 && dir   <= 1) {
		gOverridden = 0;
		speed = nordic_getWirelessPropJoySpeed();
		dir = nordic_getWirelessPropJoyDirection();
	} else {
		gOverridden = 1;
	}

	if (menuGetMotorsDisabled()) {
		speed = 0;
		dir = 0;
	}

	outerDeadBandLogic(speed, dir);
	if (gIsOuterDeadBand && gIsOuterDeadBandTimeout) {
		speed = 0;
		dir = 0;
	}

	if (menuGetInvert(gOverridden)) {
		speed = -speed;
	}

	// Proportional joystick as switch joystick
	if (menuGetPropAsSwitch(gOverridden))
	{
		uint8_t threshold = 50;
		if (speed > threshold) {
			speed = menuGetTopFwdSpeed(gOverridden);
		} else if (speed < -threshold) {
			speed = -menuGetTopRevSpeed(gOverridden);
		} else {
			speed = 0;
		}

		if (dir > threshold) {
			dir = menuGetTopTurnSpeed(gOverridden);
		} else if (dir < -threshold) {
			dir = -menuGetTopTurnSpeed(gOverridden);
		} else {
			dir = 0;
		}
	}
	else
	{
		// apply center dead band
		centerDeadBand(&dir, &speed, menuGetCenterDeadBand(gOverridden));

		// fwd/rev throw
		if (speed > 0) {
			speed *= menuGetFwdThrow(gOverridden);
		}
		if (speed < 0) {
			speed *= menuGetRevThrow(gOverridden);
		}
		// turn throw
		dir *= menuGetTurnThrow(gOverridden);

		// Top speeds
		if (speed > menuGetTopFwdSpeed(gOverridden)) {
			// max forward speed
			speed = menuGetTopFwdSpeed(gOverridden);
		} else if (speed < -menuGetTopRevSpeed(gOverridden)) {
			// max reverse speed
			speed = -menuGetTopRevSpeed(gOverridden);
		}

		// max turn speed
		if (dir > menuGetTopTurnSpeed(gOverridden)) {
			dir = menuGetTopTurnSpeed(gOverridden);
		} else if (dir < -menuGetTopTurnSpeed(gOverridden)) {
			dir = -menuGetTopTurnSpeed(gOverridden);
		}
	}

	// We don't want to get interrupted while accessing shared variables
	AVR_ENTER_CRITICAL_REGION();

	if (speed >= -1 && speed <= 1 &&
		dir   >= -1 && dir   <= 1 && !gOverridden) {
		// Buddy buttons only active if joystick not active
		if (nordic_getStudentForward() != nordic_getStudentReverse()) {
			if (nordic_getStudentForward()) {
				speed = menuGetTopFwdSpeed(gOverridden);
			} else if (nordic_getStudentReverse()) {
				speed = -menuGetTopRevSpeed(gOverridden);
			}
		}
		if (nordic_getStudentRight() != nordic_getStudentLeft()) {
			if (nordic_getStudentRight()) {
				dir = menuGetTopTurnSpeed(gOverridden);
			} else if (nordic_getStudentLeft()) {
				dir = -menuGetTopTurnSpeed(gOverridden);
			}
		}
	}

	// We've applied the direct-mapped logic, now it's time to hand it off to the filters
	gSpeedPreFilter = speed;
	gDirPreFilter = dir;

	speed = gSpeedPostFilter;
	dir = gDirPostFilter;
	AVR_LEAVE_CRITICAL_REGION();

	*returnSpeed = speed;
	*returnDir = dir;
}

// Filter topography and variable naming:
// preFilter --> [low-pass] --> betweenFilters --> [accel/decel] --> postFilter
ISR(TCD1_CCA_vect)
{
	static uint8_t accelerationCount = 0;
	static uint8_t decelerationCount = 0;
	static uint16_t outerDeadBandMillisecondCount = 0;
	static uint8_t outerDeadBandTime = 0;

	if (gIsOuterDeadBand) {
		if (!gIsOuterDeadBandTimeout) {
			uint8_t timeoutTime = menuGetOuterDeadBand(gOverridden) - 1;
			if (timeoutTime == 0) {
				gIsOuterDeadBandTimeout = 1;
			} else if (timeoutTime > 0) {
				outerDeadBandMillisecondCount++;
				if (outerDeadBandMillisecondCount >= 500) {
					outerDeadBandMillisecondCount = 0;
					outerDeadBandTime++;
					if (outerDeadBandTime >= timeoutTime) {
						gIsOuterDeadBandTimeout = 1;
					}
				}
			}
		}
	} else {
		outerDeadBandMillisecondCount = 0;
		outerDeadBandTime = 0;
		gIsOuterDeadBandTimeout = 0;
	}

	// Low-pass filter (aka Tremor Dampening aka Tremor Suppression aka Sensitivity)
	if ((gSpeedPreFilter > 0 && gSpeedPreFilter > gSpeedBetweenFilters) ||
		(gSpeedPreFilter < 0 && gSpeedPreFilter < gSpeedBetweenFilters)) {
		gSpeedBetweenFilters = gSpeedBetweenFilters + menuGetSensitivity(gOverridden) * (gSpeedPreFilter - gSpeedBetweenFilters);
	} else {
		gSpeedBetweenFilters = gSpeedPreFilter;
	}
	if ((gDirPreFilter > 0 && gDirPreFilter > gDirBetweenFilters) ||
		(gDirPreFilter < 0 && gDirPreFilter < gDirBetweenFilters)) {
		gDirBetweenFilters = gDirBetweenFilters + menuGetSensitivity(gOverridden) * (gDirPreFilter - gDirBetweenFilters);
	} else {
		gDirBetweenFilters = gDirPreFilter;
	}

	// Acceleration/deceleration: must wait X milliseconds before speed/dir is changed by 1
	accelerationCount++;
	decelerationCount++;
	if (accelerationCount >= menuGetAcceleration(gOverridden))
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

	if (decelerationCount >= menuGetDeceleration(gOverridden))
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
