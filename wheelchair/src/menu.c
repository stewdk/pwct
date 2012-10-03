/*
 * menu.c
 *
 * Created: 8/16/2012 10:19:14 AM
 *  Author: Stew
 */ 

#include <stdio.h>
#include <avr/eeprom.h>
#include "../atmel/avr_compiler.h"
#include "menu.h"
#include "PWCT_io.h"
#include "lcd_driver.h"
#include "motor_driver.h"

// Define the order of the menu options, zero-based
#define MENU_OPTION_FWD_THROW 0
#define MENU_OPTION_REV_THROW 1
#define MENU_OPTION_TURN_THROW 2
#define MENU_OPTION_TOP_FWD_SPEED 3
#define MENU_OPTION_TOP_REV_SPEED 4
#define MENU_OPTION_TOP_TURN_SPEED 5
#define MENU_OPTION_SENSITIVITY 6
#define MENU_OPTION_ACCELERATION 7
#define MENU_OPTION_DECELERATION 8
#define MENU_OPTION_OUTER_DEAD_BAND 9
#define MENU_OPTION_CTR_DEAD_BAND 10
#define MENU_OPTION_PROP_AS_SWITCH 11
#define MENU_OPTION_INVERT 12
// We must know how many menu options there are
#define LAST_MENU_OPTION MENU_OPTION_INVERT

// EEPROM variables and sane initial values
// Note: the initial values are only updated when programming the EEPROM memory (wheelchair.eep)
uint8_t EEMEM eepromMenuState = 0;
float EEMEM eepromFwdThrow = 1.0;
float EEMEM eepromRevThrow = 0.8;
float EEMEM eepromTurnThrow = 0.6;
uint8_t EEMEM eepromTopFwdSpeed = 50;
uint8_t EEMEM eepromTopRevSpeed = 35;
uint8_t EEMEM eepromTopTurnSpeed = 20;
float EEMEM eepromSensitivity = 0.004;
uint8_t EEMEM eepromAcceleration = 14;
uint8_t EEMEM eepromDeceleration = 10;
uint8_t EEMEM eepromOuterDeadBand = 0;
uint8_t EEMEM eepromCenterDeadBand = 3;
uint8_t EEMEM eepromPropAsSwitch = 0;
uint8_t EEMEM eepromInvert = 0;

uint8_t gWirelessTimeout = 0;

void incrementWirelessTimeout()
{
	AVR_ENTER_CRITICAL_REGION();
	gWirelessTimeout++;
	AVR_LEAVE_CRITICAL_REGION();
}

float menuGetFwdThrow(void)
{
	return eeprom_read_float(&eepromFwdThrow);
}

float menuGetRevThrow(void)
{
	return eeprom_read_float(&eepromRevThrow);
}

float menuGetTurnThrow(void)
{
	return eeprom_read_float(&eepromTurnThrow);
}

uint8_t menuGetTopFwdSpeed(void)
{
	return eeprom_read_byte(&eepromTopFwdSpeed);
}

uint8_t menuGetTopRevSpeed(void)
{
	return eeprom_read_byte(&eepromTopRevSpeed);
}

uint8_t menuGetTopTurnSpeed(void)
{
	return eeprom_read_byte(&eepromTopTurnSpeed);
}

float menuGetSensitivity(void)
{
	return eeprom_read_float(&eepromSensitivity);
}

uint8_t menuGetAcceleration(void)
{
	return eeprom_read_byte(&eepromAcceleration);
}

uint8_t menuGetDeceleration(void)
{
	return eeprom_read_byte(&eepromDeceleration);
}

uint8_t menuGetOuterDeadBand(void)
{
	return eeprom_read_byte(&eepromOuterDeadBand);
}

uint8_t menuGetCenterDeadBand(void)
{
	return eeprom_read_byte(&eepromCenterDeadBand);
}

uint8_t menuGetPropAsSwitch(void)
{
	return eeprom_read_byte(&eepromPropAsSwitch);
}

uint8_t menuGetInvert(void)
{
	return eeprom_read_byte(&eepromInvert);
}

void menuUpdate(int16_t speed, int16_t dir)
{
	uint8_t up = lcdUpFallingEdge();
	uint8_t down = lcdDownFallingEdge();
	uint8_t right = lcdRightFallingEdge();
	uint8_t left = lcdLeftFallingEdge();

	if (right)
	{
		if (eeprom_read_byte(&eepromMenuState) < LAST_MENU_OPTION) {
			eeprom_update_byte(&eepromMenuState, eeprom_read_byte(&eepromMenuState) + 1);
			printf("EEPROM written\n");
		}
	}
	if (left)
	{
		if (eeprom_read_byte(&eepromMenuState) > 0) {
			eeprom_update_byte(&eepromMenuState, eeprom_read_byte(&eepromMenuState) - 1);
			printf("EEPROM written\n");
		}
	}

	char lcdLine1[17];
	char lcdLine2[17];
	lcdLine1[16] = '\0';
	lcdLine2[16] = '\0';

	switch (eeprom_read_byte(&eepromMenuState))
	{
	case MENU_OPTION_FWD_THROW:
		if (up && menuGetFwdThrow() < 2.45) {
			eeprom_update_float(&eepromFwdThrow, menuGetFwdThrow() + 0.05);
			printf("EEPROM written\n");
		}
		if (down && menuGetFwdThrow() > 0.05) {
			eeprom_update_float(&eepromFwdThrow, menuGetFwdThrow() - 0.05);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "FwdThrow=%.2f", (double)menuGetFwdThrow());
		break;
	case MENU_OPTION_REV_THROW:
		if (up && menuGetRevThrow() < 2.45) {
			eeprom_update_float(&eepromRevThrow, menuGetRevThrow() + 0.05);
			printf("EEPROM written\n");
		}
		if (down && menuGetRevThrow() > 0.05) {
			eeprom_update_float(&eepromRevThrow, menuGetRevThrow() - 0.05);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "RevThrow=%.2f", (double)menuGetRevThrow());
		break;
	case MENU_OPTION_TURN_THROW:
		if (up && menuGetTurnThrow() < 2.45) {
			eeprom_update_float(&eepromTurnThrow, menuGetTurnThrow() + 0.05);
			printf("EEPROM written\n");
		}
		if (down && menuGetTurnThrow() > 0.05) {
			eeprom_update_float(&eepromTurnThrow, menuGetTurnThrow() - 0.05);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "TurnThrow=%.2f", (double)menuGetTurnThrow());
		break;
	case MENU_OPTION_TOP_FWD_SPEED:
		if (up) {
			eeprom_update_byte(&eepromTopFwdSpeed, menuGetTopFwdSpeed() + 5);
			printf("EEPROM written\n");
		}
		if (down) {
			eeprom_update_byte(&eepromTopFwdSpeed, menuGetTopFwdSpeed() - 5);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "TopFwdSpd=%d", menuGetTopFwdSpeed());
		break;
	case MENU_OPTION_TOP_REV_SPEED:
		if (up) {
			eeprom_update_byte(&eepromTopRevSpeed, menuGetTopRevSpeed() + 5);
			printf("EEPROM written\n");
		}
		if (down) {
			eeprom_update_byte(&eepromTopRevSpeed, menuGetTopRevSpeed() - 5);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "TopRevSpd=%d", menuGetTopRevSpeed());
		break;
	case MENU_OPTION_TOP_TURN_SPEED:
		if (up) {
			eeprom_update_byte(&eepromTopTurnSpeed, menuGetTopTurnSpeed() + 5);
			printf("EEPROM written\n");
		}
		if (down) {
			eeprom_update_byte(&eepromTopTurnSpeed, menuGetTopTurnSpeed() - 5);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "TopTurnSpd=%d", menuGetTopTurnSpeed());
		break;
	case MENU_OPTION_SENSITIVITY:
		if (up && menuGetSensitivity() < 0.5) {
			eeprom_update_float(&eepromSensitivity, menuGetSensitivity() * 2);
			printf("EEPROM written\n");
		}
		if (down && menuGetSensitivity() > 0.0001) {
			eeprom_update_float(&eepromSensitivity, menuGetSensitivity() / 2);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "Sens.=%1.4f", (double)menuGetSensitivity());
		break;
	case MENU_OPTION_ACCELERATION:
		// Todo: numbers go opposite way (i.e. higher number means accelerate quicker)
		// Also, impose limits
		if (up) {
			eeprom_update_byte(&eepromAcceleration, menuGetAcceleration() + 2);
			printf("EEPROM written\n");
		}
		if (down) {
			eeprom_update_byte(&eepromAcceleration, menuGetAcceleration() - 2);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "Acceleration=%d", menuGetAcceleration());
		break;
	case MENU_OPTION_DECELERATION:
		if (up) {
			eeprom_update_byte(&eepromDeceleration, menuGetDeceleration() + 2);
			printf("EEPROM written\n");
		}
		if (down) {
			eeprom_update_byte(&eepromDeceleration, menuGetDeceleration() - 2);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "Deceleration=%d", menuGetDeceleration());
		break;
	case MENU_OPTION_OUTER_DEAD_BAND:
		if (up && menuGetOuterDeadBand() < 20) {
			eeprom_update_byte(&eepromOuterDeadBand, menuGetOuterDeadBand() + 1);
			printf("EEPROM written\n");
		}
		if (down && menuGetOuterDeadBand() > 0) {
			eeprom_update_byte(&eepromOuterDeadBand, menuGetOuterDeadBand() - 1);
			printf("EEPROM written\n");
		}
		if (menuGetOuterDeadBand() == 0) {
			// 0: off
			sprintf(lcdLine1, "OuterDB=Off");
		} else if (menuGetOuterDeadBand() == 1) {
			// 1: immediate
			sprintf(lcdLine1, "OuterDB=Immed");
		} else {
			// 2: 0.5s, 3: 1.0s, 4: 1.5s, etc
			// Conversion: y=(x-1)/2
			sprintf(lcdLine1, "OuterDB=%d.%ds", (menuGetOuterDeadBand()-1)/2, (menuGetOuterDeadBand()-1) % 2 ? 5 : 0);
		}
		break;
	case MENU_OPTION_CTR_DEAD_BAND:
		if (up && menuGetCenterDeadBand() < 25) {
			eeprom_update_byte(&eepromCenterDeadBand, menuGetCenterDeadBand() + 1);
			printf("EEPROM written\n");
		}
		if (down && menuGetCenterDeadBand() > 0) {
			eeprom_update_byte(&eepromCenterDeadBand, menuGetCenterDeadBand() - 1);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "Ctr DeadBand=%d", menuGetCenterDeadBand());
		break;
	case MENU_OPTION_PROP_AS_SWITCH:
		if (up || down) {
			if (menuGetPropAsSwitch()) {
				eeprom_update_byte(&eepromPropAsSwitch, 0);
			} else {
				eeprom_update_byte(&eepromPropAsSwitch, 1);
			}
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "PropAsSwitch=%s", menuGetPropAsSwitch() ? "On" : "Off");
		break;
	case MENU_OPTION_INVERT:
		if (up || down) {
			if (menuGetInvert()) {
				eeprom_update_byte(&eepromInvert, 0);
			} else {
				eeprom_update_byte(&eepromInvert, 1);
			}
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "Invert=%s", menuGetInvert() ? "On" : "Off");
		break;
	default:
		lcdLine1[0] = '\0';
		break;
	}
	sprintf(lcdLine2, "S=%4d T=%4d%3d", speed, dir, gWirelessTimeout);

	lcdText(lcdLine1, lcdLine2, 0);
}
