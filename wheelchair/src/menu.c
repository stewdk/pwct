/*
 * menu.c
 *
 * Created: 8/16/2012 10:19:14 AM
 *  Author: Stew
 */ 

#include <stdio.h>
#include <avr/eeprom.h>
#include "../atmel/avr_compiler.h"
#include "../atmel/wdt_driver.h"
#include "menu.h"
#include "PWCT_io.h"
#include "lcd_driver.h"
#include "motor_driver.h"

// Define the order of the menu options, zero-based
#define MENU_OPTION_PROFILE 0
#define MENU_OPTION_FWD_THROW 1
#define MENU_OPTION_REV_THROW 2
#define MENU_OPTION_TURN_THROW 3
#define MENU_OPTION_TOP_FWD_SPEED 4
#define MENU_OPTION_TOP_REV_SPEED 5
#define MENU_OPTION_TOP_TURN_SPEED 6
#define MENU_OPTION_SENSITIVITY 7
#define MENU_OPTION_ACCELERATION 8
#define MENU_OPTION_DECELERATION 9
#define MENU_OPTION_OUTER_DEAD_BAND 10
#define MENU_OPTION_PROP_AS_SWITCH 11
#define MENU_OPTION_INVERT 12
#define MENU_OPTION_CTR_DEAD_BAND 13
// We must know how many menu options there are
#define LAST_MENU_OPTION MENU_OPTION_CTR_DEAD_BAND


uint8_t gWirelessTimeoutCount = 0;

void incrementWirelessTimeout()
{
	gWirelessTimeoutCount++;
}

// EEPROM variables, RAM shadow variables, and sane initial values
// Note: the initial values are only updated when programming the EEPROM memory (wheelchair.eep)

#define PROFILE_COUNT 20

uint8_t EEMEM eepromIsPlatformDown = 0;
uint8_t eepromShadowIsPlatformDown = 0;

uint8_t EEMEM eepromCurrentProfile = 0;
uint8_t eepromShadowCurrentProfile = 0;

uint8_t EEMEM eepromMenuState = 0;
uint8_t eepromShadowMenuState = 0;

float EEMEM eepromFwdThrow[PROFILE_COUNT] = {1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.5};
float eepromShadowFwdThrow = 1.0;

float EEMEM eepromRevThrow[PROFILE_COUNT] = {0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 1.0};
float eepromShadowRevThrow = 0.8;

float EEMEM eepromTurnThrow[PROFILE_COUNT] = {0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.8};
float eepromShadowTurnThrow = 0.6;

uint8_t EEMEM eepromTopFwdSpeed[PROFILE_COUNT] = {50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 50, 125};
uint8_t eepromShadowTopFwdSpeed = 50;

uint8_t EEMEM eepromTopRevSpeed[PROFILE_COUNT] = {35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 35, 50};
uint8_t eepromShadowTopRevSpeed = 35;

uint8_t EEMEM eepromTopTurnSpeed[PROFILE_COUNT] = {20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 20, 35};
uint8_t eepromShadowTopTurnSpeed = 20;

float EEMEM eepromSensitivity[PROFILE_COUNT] = {0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.008, 0.256};
float eepromShadowSensitivity = 0.004;

uint8_t EEMEM eepromAcceleration[PROFILE_COUNT] = {16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
uint8_t eepromShadowAcceleration = 16;

uint8_t EEMEM eepromDeceleration[PROFILE_COUNT] = {12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};
uint8_t eepromShadowDeceleration = 12;

uint8_t EEMEM eepromOuterDeadBand[PROFILE_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t eepromShadowOuterDeadBand = 0;

uint8_t EEMEM eepromCenterDeadBand[PROFILE_COUNT] = {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3};
uint8_t eepromShadowCenterDeadBand = 3;

uint8_t EEMEM eepromPropAsSwitch[PROFILE_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t eepromShadowPropAsSwitch = 0;

uint8_t EEMEM eepromInvert[PROFILE_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t eepromShadowInvert = 0;

static void eepromCorrupt()
{
	motorEStop();
	while (1)
	{
		lcdText("EEPROM corrupt", "Ver. " __DATE__, 0);
		WDT_Reset();
	}
}

void menuInit()
{
	// Initialize all shadow variables
	eepromShadowIsPlatformDown = eeprom_read_byte(&eepromIsPlatformDown);
	eepromShadowCurrentProfile = eeprom_read_byte(&eepromCurrentProfile);
	if (eepromShadowCurrentProfile >= PROFILE_COUNT) {
		eepromCorrupt();
	}
	eepromShadowMenuState = eeprom_read_byte(&eepromMenuState);
	eepromShadowFwdThrow = eeprom_read_float(&eepromFwdThrow[eepromShadowCurrentProfile]);
	eepromShadowRevThrow = eeprom_read_float(&eepromRevThrow[eepromShadowCurrentProfile]);
	eepromShadowTurnThrow = eeprom_read_float(&eepromTurnThrow[eepromShadowCurrentProfile]);
	eepromShadowTopFwdSpeed = eeprom_read_byte(&eepromTopFwdSpeed[eepromShadowCurrentProfile]);
	eepromShadowTopRevSpeed = eeprom_read_byte(&eepromTopRevSpeed[eepromShadowCurrentProfile]);
	eepromShadowTopTurnSpeed = eeprom_read_byte(&eepromTopTurnSpeed[eepromShadowCurrentProfile]);
	eepromShadowSensitivity = eeprom_read_float(&eepromSensitivity[eepromShadowCurrentProfile]);
	eepromShadowAcceleration = eeprom_read_byte(&eepromAcceleration[eepromShadowCurrentProfile]);
	eepromShadowDeceleration = eeprom_read_byte(&eepromDeceleration[eepromShadowCurrentProfile]);
	eepromShadowOuterDeadBand = eeprom_read_byte(&eepromOuterDeadBand[eepromShadowCurrentProfile]);
	eepromShadowCenterDeadBand = eeprom_read_byte(&eepromCenterDeadBand[eepromShadowCurrentProfile]);
	eepromShadowPropAsSwitch = eeprom_read_byte(&eepromPropAsSwitch[eepromShadowCurrentProfile]);
	eepromShadowInvert = eeprom_read_byte(&eepromInvert[eepromShadowCurrentProfile]);
}

static void eepromUpdateByteSafe(uint8_t *eepromVariable, uint8_t *shadowVariable, uint8_t newValue)
{
	uint8_t readValue;
	eeprom_busy_wait();
	eeprom_update_byte(eepromVariable, newValue);
	printf("EEPROM written\n");
	eeprom_busy_wait();
	readValue = eeprom_read_byte(eepromVariable);
	if (readValue != newValue) {
		eepromCorrupt();
	} else {
		*shadowVariable = newValue;
	}
}

static void eepromUpdateFloatSafe(float *eepromVariable, float *shadowVariable, float newValue)
{
	float readValue;
	eeprom_busy_wait();
	eeprom_update_float(eepromVariable, newValue);
	printf("EEPROM written\n");
	eeprom_busy_wait();
	readValue = eeprom_read_float(eepromVariable);
	if (readValue != newValue) {
		eepromCorrupt();
	} else {
		*shadowVariable = newValue;
	}
}

void menuPlatformDownPushed() {
	if (!eepromShadowIsPlatformDown) {
		eepromUpdateByteSafe(&eepromIsPlatformDown, &eepromShadowIsPlatformDown, 1);
	}
}

void menuPlatformUpPushed() {
	if (eepromShadowIsPlatformDown) {
		eepromUpdateByteSafe(&eepromIsPlatformDown, &eepromShadowIsPlatformDown, 0);
	}
}

uint8_t menuGetIsPlatformDown() {
	return eepromShadowIsPlatformDown;
}

float menuGetFwdThrow(void)
{
	return eepromShadowFwdThrow;
}

float menuGetRevThrow(void)
{
	return eepromShadowRevThrow;
}

float menuGetTurnThrow(void)
{
	return eepromShadowTurnThrow;
}

uint8_t menuGetTopFwdSpeed(void)
{
	return eepromShadowTopFwdSpeed;
}

uint8_t menuGetTopRevSpeed(void)
{
	return eepromShadowTopRevSpeed;
}

uint8_t menuGetTopTurnSpeed(void)
{
	return eepromShadowTopTurnSpeed;
}

float menuGetSensitivity(void)
{
	return eepromShadowSensitivity;
}

uint8_t menuGetAcceleration(void)
{
	return eepromShadowAcceleration;
}

uint8_t menuGetDeceleration(void)
{
	return eepromShadowDeceleration;
}

uint8_t menuGetOuterDeadBand(void)
{
	return eepromShadowOuterDeadBand;
}

uint8_t menuGetCenterDeadBand(void)
{
	return eepromShadowCenterDeadBand;
}

uint8_t menuGetPropAsSwitch(void)
{
	return eepromShadowPropAsSwitch;
}

uint8_t menuGetInvert(void)
{
	return eepromShadowInvert;
}

void menuUpdate(int16_t speed, int16_t dir)
{
	uint8_t up = lcdUpFallingEdge();
	uint8_t down = lcdDownFallingEdge();
	uint8_t right = lcdRightFallingEdge();
	uint8_t left = lcdLeftFallingEdge();

	char lcdLine1[17];
	char lcdLine2[17];
	lcdLine1[16] = '\0';
	lcdLine2[16] = '\0';

	if (eepromShadowIsPlatformDown) {
		sprintf(lcdLine1, "Platform down");
		lcdLine2[0] = '\0';
		lcdText(lcdLine1, lcdLine2, 0);
		return;
	}

	if (right)
	{
		if (eepromShadowMenuState < LAST_MENU_OPTION) {
			eepromUpdateByteSafe(&eepromMenuState, &eepromShadowMenuState, eepromShadowMenuState + 1);
		}
	}
	if (left)
	{
		if (eepromShadowMenuState > 0) {
			eepromUpdateByteSafe(&eepromMenuState, &eepromShadowMenuState, eepromShadowMenuState - 1);
		}
	}

	switch (eepromShadowMenuState)
	{
	case MENU_OPTION_PROFILE:
		if (up && eepromShadowCurrentProfile < PROFILE_COUNT - 1) {
			eepromUpdateByteSafe(&eepromCurrentProfile, &eepromShadowCurrentProfile, eepromShadowCurrentProfile + 1);
		}
		if (down && eepromShadowCurrentProfile > 0) {
			eepromUpdateByteSafe(&eepromCurrentProfile, &eepromShadowCurrentProfile, eepromShadowCurrentProfile - 1);
		}
		sprintf(lcdLine1, "Profile=%d", eepromShadowCurrentProfile + 1);
		menuInit();
		break;
	case MENU_OPTION_FWD_THROW:
		if (up && eepromShadowFwdThrow < 2.45) {
			eepromUpdateFloatSafe(&eepromFwdThrow[eepromShadowCurrentProfile], &eepromShadowFwdThrow, eepromShadowFwdThrow + 0.05);
		}
		if (down && eepromShadowFwdThrow > 0.05) {
			eepromUpdateFloatSafe(&eepromFwdThrow[eepromShadowCurrentProfile], &eepromShadowFwdThrow, eepromShadowFwdThrow - 0.05);
		}
		sprintf(lcdLine1, "FwdThrow=%.2f", (double)eepromShadowFwdThrow);
		break;
	case MENU_OPTION_REV_THROW:
		if (up && eepromShadowRevThrow < 2.45) {
			eepromUpdateFloatSafe(&eepromRevThrow[eepromShadowCurrentProfile], &eepromShadowRevThrow, eepromShadowRevThrow + 0.05);
		}
		if (down && eepromShadowRevThrow > 0.05) {
			eepromUpdateFloatSafe(&eepromRevThrow[eepromShadowCurrentProfile], &eepromShadowRevThrow, eepromShadowRevThrow - 0.05);
		}
		sprintf(lcdLine1, "RevThrow=%.2f", (double)eepromShadowRevThrow);
		break;
	case MENU_OPTION_TURN_THROW:
		if (up && eepromShadowTurnThrow < 2.45) {
			eepromUpdateFloatSafe(&eepromTurnThrow[eepromShadowCurrentProfile], &eepromShadowTurnThrow, eepromShadowTurnThrow + 0.05);
		}
		if (down && eepromShadowTurnThrow > 0.05) {
			eepromUpdateFloatSafe(&eepromTurnThrow[eepromShadowCurrentProfile], &eepromShadowTurnThrow, eepromShadowTurnThrow - 0.05);
		}
		sprintf(lcdLine1, "TurnThrow=%.2f", (double)eepromShadowTurnThrow);
		break;
	case MENU_OPTION_TOP_FWD_SPEED:
		if (up && eepromShadowTopFwdSpeed < 125) {
			eepromUpdateByteSafe(&eepromTopFwdSpeed[eepromShadowCurrentProfile], &eepromShadowTopFwdSpeed, eepromShadowTopFwdSpeed + 5);
		}
		if (down && eepromShadowTopFwdSpeed > 5) {
			eepromUpdateByteSafe(&eepromTopFwdSpeed[eepromShadowCurrentProfile], &eepromShadowTopFwdSpeed, eepromShadowTopFwdSpeed - 5);
		}
		sprintf(lcdLine1, "TopFwdSpd=%d", eepromShadowTopFwdSpeed);
		break;
	case MENU_OPTION_TOP_REV_SPEED:
		if (up && eepromShadowTopRevSpeed < 125) {
			eepromUpdateByteSafe(&eepromTopRevSpeed[eepromShadowCurrentProfile], &eepromShadowTopRevSpeed, eepromShadowTopRevSpeed + 5);
		}
		if (down && eepromShadowTopRevSpeed > 5) {
			eepromUpdateByteSafe(&eepromTopRevSpeed[eepromShadowCurrentProfile], &eepromShadowTopRevSpeed, eepromShadowTopRevSpeed - 5);
		}
		sprintf(lcdLine1, "TopRevSpd=%d", eepromShadowTopRevSpeed);
		break;
	case MENU_OPTION_TOP_TURN_SPEED:
		if (up && eepromShadowTopTurnSpeed < 125) {
			eepromUpdateByteSafe(&eepromTopTurnSpeed[eepromShadowCurrentProfile], &eepromShadowTopTurnSpeed, eepromShadowTopTurnSpeed + 5);
		}
		if (down && eepromShadowTopTurnSpeed > 5) {
			eepromUpdateByteSafe(&eepromTopTurnSpeed[eepromShadowCurrentProfile], &eepromShadowTopTurnSpeed, eepromShadowTopTurnSpeed - 5);
		}
		sprintf(lcdLine1, "TopTurnSpd=%d", eepromShadowTopTurnSpeed);
		break;
	case MENU_OPTION_SENSITIVITY:
		if (up && eepromShadowSensitivity < 0.5) {
			eepromUpdateFloatSafe(&eepromSensitivity[eepromShadowCurrentProfile], &eepromShadowSensitivity, eepromShadowSensitivity * 2);
		}
		if (down && eepromShadowSensitivity > 0.0002) {
			eepromUpdateFloatSafe(&eepromSensitivity[eepromShadowCurrentProfile], &eepromShadowSensitivity, eepromShadowSensitivity / 2);
		}
		sprintf(lcdLine1, "Sens.=%.4f", (double)eepromShadowSensitivity);
		break;
	case MENU_OPTION_ACCELERATION:
		if (up && eepromShadowAcceleration > 4) {
			eepromUpdateByteSafe(&eepromAcceleration[eepromShadowCurrentProfile], &eepromShadowAcceleration, eepromShadowAcceleration - 4);
		}
		if (down && eepromShadowAcceleration < 100) {
			eepromUpdateByteSafe(&eepromAcceleration[eepromShadowCurrentProfile], &eepromShadowAcceleration, eepromShadowAcceleration + 4);
		}
		sprintf(lcdLine1, "Acceleration=%d", (104 - eepromShadowAcceleration) / 4);
		break;
	case MENU_OPTION_DECELERATION:
		if (up && eepromShadowDeceleration > 4) {
			eepromUpdateByteSafe(&eepromDeceleration[eepromShadowCurrentProfile], &eepromShadowDeceleration, eepromShadowDeceleration - 4);
		}
		if (down && eepromShadowDeceleration < 100) {
			eepromUpdateByteSafe(&eepromDeceleration[eepromShadowCurrentProfile], &eepromShadowDeceleration, eepromShadowDeceleration + 4);
		}
		sprintf(lcdLine1, "Deceleration=%d", (104 - eepromShadowDeceleration) / 4);
		break;
	case MENU_OPTION_OUTER_DEAD_BAND:
		if (up && eepromShadowOuterDeadBand < 20) {
			eepromUpdateByteSafe(&eepromOuterDeadBand[eepromShadowCurrentProfile], &eepromShadowOuterDeadBand, eepromShadowOuterDeadBand + 1);
		}
		if (down && eepromShadowOuterDeadBand > 0) {
			eepromUpdateByteSafe(&eepromOuterDeadBand[eepromShadowCurrentProfile], &eepromShadowOuterDeadBand, eepromShadowOuterDeadBand - 1);
		}
		if (eepromShadowOuterDeadBand == 0) {
			// 0: off
			sprintf(lcdLine1, "OuterDB=Off");
		} else if (eepromShadowOuterDeadBand == 1) {
			// 1: immediate
			sprintf(lcdLine1, "OuterDB=Immed");
		} else {
			// 2: 0.5s, 3: 1.0s, 4: 1.5s, etc
			// Conversion: y=(x-1)/2
			sprintf(lcdLine1, "OuterDB=%d.%ds", (eepromShadowOuterDeadBand-1)/2, (eepromShadowOuterDeadBand-1) % 2 ? 5 : 0);
		}
		break;
	case MENU_OPTION_CTR_DEAD_BAND:
		if (up && eepromShadowCenterDeadBand < 10) {
			eepromUpdateByteSafe(&eepromCenterDeadBand[eepromShadowCurrentProfile], &eepromShadowCenterDeadBand, eepromShadowCenterDeadBand + 1);
		}
		if (down && eepromShadowCenterDeadBand > 0) {
			eepromUpdateByteSafe(&eepromCenterDeadBand[eepromShadowCurrentProfile], &eepromShadowCenterDeadBand, eepromShadowCenterDeadBand - 1);
		}
		sprintf(lcdLine1, "Ctr DeadBand=%d", eepromShadowCenterDeadBand);
		break;
	case MENU_OPTION_PROP_AS_SWITCH:
		if (up || down) {
			if (eepromShadowPropAsSwitch) {
				eepromUpdateByteSafe(&eepromPropAsSwitch[eepromShadowCurrentProfile], &eepromShadowPropAsSwitch, 0);
			} else {
				eepromUpdateByteSafe(&eepromPropAsSwitch[eepromShadowCurrentProfile], &eepromShadowPropAsSwitch, 1);
			}
		}
		sprintf(lcdLine1, "PropAsSwitch=%s", eepromShadowPropAsSwitch ? "On" : "Off");
		break;
	case MENU_OPTION_INVERT:
		if (up || down) {
			if (eepromShadowInvert) {
				eepromUpdateByteSafe(&eepromInvert[eepromShadowCurrentProfile], &eepromShadowInvert, 0);
			} else {
				eepromUpdateByteSafe(&eepromInvert[eepromShadowCurrentProfile], &eepromShadowInvert, 1);
			}
		}
		sprintf(lcdLine1, "Invert=%s", eepromShadowInvert ? "On" : "Off");
		break;
	default:
		lcdLine1[0] = '\0';
		break;
	}
	//sprintf(lcdLine2, "S=%4d T=%4d%3d", speed, dir, gWirelessTimeoutCount);
	sprintf(lcdLine2, "S=%4d T=%4d", speed, dir);

	lcdText(lcdLine1, lcdLine2, 0);
}
