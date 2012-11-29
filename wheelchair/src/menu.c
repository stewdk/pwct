/*
 * menu.c
 *
 * Created: 8/16/2012 10:19:14 AM
 *  Author: Stew
 */ 

#include <stdio.h>
#include <string.h>
#include <avr/eeprom.h>
#include "../atmel/avr_compiler.h"
#include "../atmel/wdt_driver.h"
#include "menu.h"
#include "PWCT_io.h"
#include "lcd_driver.h"
#include "motor_driver.h"

uint8_t gWirelessTimeoutCount = 0;
uint8_t gMotorsDisabled = 0;
uint8_t gNameEditMode = 0;

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
#define MENU_OPTION_CTR_DEAD_BAND 10
#define MENU_OPTION_OUTER_DEAD_BAND 11
#define MENU_OPTION_INVERT 12
#define MENU_OPTION_PROP_AS_SWITCH 13
// We must know how many menu options there are
#define LAST_MENU_OPTION MENU_OPTION_PROP_AS_SWITCH

// EEPROM variables, RAM shadow variables, and sane initial values
// Note: the initial values are only updated when programming the EEPROM memory (wheelchair.eep)

#define PROFILE_COUNT 21

uint8_t EEMEM eepromIsPlatformDown = 0;
uint8_t eepromShadowIsPlatformDown = 0;

uint8_t EEMEM eepromCurrentProfile = 0;
uint8_t eepromShadowCurrentProfile = 0;

uint8_t EEMEM eepromMenuState = 0;
uint8_t eepromShadowMenuState = 0;

char EEMEM eepromProfileName[PROFILE_COUNT][LCD_NUM_CHARACTERS+1] = {"Profile 1", "Profile 2", "Profile 3", "Profile 4", "Profile 5",
	"Profile 6", "Profile 7", "Profile 8", "Profile 9", "Profile 10", "Profile 11", "Profile 12", "Profile 13",
	"Profile 14", "Profile 15", "Profile 16", "Profile 17", "Profile 18", "Profile 19", "Profile 20", "Remote override"};
char currentProfileName[LCD_NUM_CHARACTERS+1];

float EEMEM eepromFwdThrow[PROFILE_COUNT] = {1.0, 1.0, 2.0, 1.0, 1.0, 0.65, 0.75, 1.0, 1.05, 1.0, 0.8, 1.0, 1.05, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 2.5, 0.8};
float eepromShadowFwdThrow = 1.0;
float eepromShadowOverrideFwdThrow = 0.8;

float EEMEM eepromRevThrow[PROFILE_COUNT] = {0.75, 0.8, 0.8, 0.8, 0.8, 0.8, 0.75, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 0.8, 1.0, 0.6};
float eepromShadowRevThrow = 0.8;
float eepromShadowOverrideRevThrow = 0.6;

float EEMEM eepromTurnThrow[PROFILE_COUNT] = {0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.85, 0.6, 0.55, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.6, 0.8, 0.4};
float eepromShadowTurnThrow = 0.6;
float eepromShadowOverrideTurnThrow = 0.4;

uint8_t EEMEM eepromTopFwdSpeed[PROFILE_COUNT] = {40, 40, 35, 35, 45, 90, 15, 35, 45, 40, 45, 25, 50, 50, 50, 50, 50, 50, 50, 125, 50};
uint8_t eepromShadowTopFwdSpeed = 50;
uint8_t eepromShadowOverrideTopFwdSpeed = 50;

uint8_t EEMEM eepromTopRevSpeed[PROFILE_COUNT] = {25, 35, 30, 20, 20, 35, 40, 35, 25, 35, 35, 35, 30, 35, 35, 35, 35, 35, 35, 50, 35};
uint8_t eepromShadowTopRevSpeed = 35;
uint8_t eepromShadowOverrideTopRevSpeed = 35;

uint8_t EEMEM eepromTopTurnSpeed[PROFILE_COUNT] = {25, 25, 45, 30, 35, 45, 25, 35, 35, 30, 25, 35, 25, 30, 20, 20, 20, 20, 20, 35, 35};
uint8_t eepromShadowTopTurnSpeed = 20;
uint8_t eepromShadowOverrideTopTurnSpeed = 35;

uint8_t EEMEM eepromSensitivity[PROFILE_COUNT] = {9, 7, 4, 8, 5, 8, 8, 7, 6, 5, 6, 6, 7, 9, 3, 3, 3, 3, 3, 7, 7};
uint8_t eepromShadowSensitivity = 3;
uint8_t eepromShadowOverrideSensitivity = 7;
static const double gSensitivityMap[10] = {0.0001, 0.000167, 0.000278, 0.000463, 0.000772, 0.00129, 0.00214, 0.00357, 0.01, 0.5};

uint8_t EEMEM eepromAcceleration[PROFILE_COUNT] = {16, 8, 16, 8, 4, 4, 4, 8, 8, 16, 4, 16, 16, 16, 16, 16, 16, 16, 16, 16, 16};
uint8_t eepromShadowAcceleration = 16;
uint8_t eepromShadowOverrideAcceleration = 16;

uint8_t EEMEM eepromDeceleration[PROFILE_COUNT] = {12, 12, 12, 12, 12, 12, 12, 12, 8, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12, 12};
uint8_t eepromShadowDeceleration = 12;
uint8_t eepromShadowOverrideDeceleration = 12;

uint8_t EEMEM eepromOuterDeadBand[PROFILE_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t eepromShadowOuterDeadBand = 0;
uint8_t eepromShadowOverrideOuterDeadBand = 0;

uint8_t EEMEM eepromCenterDeadBand[PROFILE_COUNT] = {2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2, 2};
uint8_t eepromShadowCenterDeadBand = 2;
uint8_t eepromShadowOverrideCenterDeadBand = 2;

uint8_t EEMEM eepromPropAsSwitch[PROFILE_COUNT] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t eepromShadowPropAsSwitch = 0;
uint8_t eepromShadowOverridePropAsSwitch = 0;

uint8_t EEMEM eepromInvert[PROFILE_COUNT] = {0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
uint8_t eepromShadowInvert = 0;
uint8_t eepromShadowOverrideInvert = 0;

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
	eeprom_read_block(currentProfileName, eepromProfileName[eepromShadowCurrentProfile], LCD_NUM_CHARACTERS+1);

	eepromShadowFwdThrow = eeprom_read_float(&eepromFwdThrow[eepromShadowCurrentProfile]);
	eepromShadowOverrideFwdThrow = eeprom_read_float(&eepromFwdThrow[PROFILE_COUNT-1]);

	eepromShadowRevThrow = eeprom_read_float(&eepromRevThrow[eepromShadowCurrentProfile]);
	eepromShadowOverrideRevThrow = eeprom_read_float(&eepromRevThrow[PROFILE_COUNT-1]);

	eepromShadowTurnThrow = eeprom_read_float(&eepromTurnThrow[eepromShadowCurrentProfile]);
	eepromShadowOverrideTurnThrow = eeprom_read_float(&eepromTurnThrow[PROFILE_COUNT-1]);

	eepromShadowTopFwdSpeed = eeprom_read_byte(&eepromTopFwdSpeed[eepromShadowCurrentProfile]);
	eepromShadowOverrideTopFwdSpeed = eeprom_read_byte(&eepromTopFwdSpeed[PROFILE_COUNT-1]);

	eepromShadowTopRevSpeed = eeprom_read_byte(&eepromTopRevSpeed[eepromShadowCurrentProfile]);
	eepromShadowOverrideTopRevSpeed = eeprom_read_byte(&eepromTopRevSpeed[PROFILE_COUNT-1]);

	eepromShadowTopTurnSpeed = eeprom_read_byte(&eepromTopTurnSpeed[eepromShadowCurrentProfile]);
	eepromShadowOverrideTopTurnSpeed = eeprom_read_byte(&eepromTopTurnSpeed[PROFILE_COUNT-1]);

	eepromShadowSensitivity = eeprom_read_byte(&eepromSensitivity[eepromShadowCurrentProfile]);
	if (eepromShadowSensitivity >= 10) {
		eepromCorrupt();
	}
	eepromShadowOverrideSensitivity = eeprom_read_byte(&eepromSensitivity[PROFILE_COUNT-1]);

	eepromShadowAcceleration = eeprom_read_byte(&eepromAcceleration[eepromShadowCurrentProfile]);
	eepromShadowOverrideAcceleration = eeprom_read_byte(&eepromAcceleration[PROFILE_COUNT-1]);

	eepromShadowDeceleration = eeprom_read_byte(&eepromDeceleration[eepromShadowCurrentProfile]);
	eepromShadowOverrideDeceleration = eeprom_read_byte(&eepromDeceleration[PROFILE_COUNT-1]);

	eepromShadowOuterDeadBand = eeprom_read_byte(&eepromOuterDeadBand[eepromShadowCurrentProfile]);
	eepromShadowOverrideOuterDeadBand = eeprom_read_byte(&eepromOuterDeadBand[PROFILE_COUNT-1]);

	eepromShadowCenterDeadBand = eeprom_read_byte(&eepromCenterDeadBand[eepromShadowCurrentProfile]);
	eepromShadowOverrideCenterDeadBand = eeprom_read_byte(&eepromCenterDeadBand[PROFILE_COUNT-1]);

	eepromShadowPropAsSwitch = eeprom_read_byte(&eepromPropAsSwitch[eepromShadowCurrentProfile]);
	eepromShadowOverridePropAsSwitch = eeprom_read_byte(&eepromPropAsSwitch[PROFILE_COUNT-1]);

	eepromShadowInvert = eeprom_read_byte(&eepromInvert[eepromShadowCurrentProfile]);
	eepromShadowOverrideInvert = eeprom_read_byte(&eepromInvert[PROFILE_COUNT-1]);
}

void incrementWirelessTimeout()
{
	gWirelessTimeoutCount++;
}

uint8_t menuGetMotorsDisabled()
{
	return gMotorsDisabled;
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

static void eepromUpdateStringSafe(char *src, char *eepromDst)
{
	char readString[LCD_NUM_CHARACTERS+1];
	eeprom_busy_wait();
	eeprom_update_block(src, eepromDst, LCD_NUM_CHARACTERS+1);
	printf("EEPROM written\n");
	eeprom_busy_wait();
	eeprom_read_block(readString, eepromDst, LCD_NUM_CHARACTERS+1);
	if (strcmp(readString, src)) {
		eepromCorrupt();
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

float menuGetFwdThrow(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideFwdThrow;
	else
		return eepromShadowFwdThrow;
}

float menuGetRevThrow(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideRevThrow;
	else
		return eepromShadowRevThrow;
}

float menuGetTurnThrow(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideTurnThrow;
	else
		return eepromShadowTurnThrow;
}

uint8_t menuGetTopFwdSpeed(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideTopFwdSpeed;
	else
		return eepromShadowTopFwdSpeed;
}

uint8_t menuGetTopRevSpeed(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideTopRevSpeed;
	else
		return eepromShadowTopRevSpeed;
}

uint8_t menuGetTopTurnSpeed(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideTopTurnSpeed;
	else
		return eepromShadowTopTurnSpeed;
}

double menuGetSensitivity(uint8_t overridden)
{
	if (overridden)
		return gSensitivityMap[eepromShadowOverrideSensitivity];
	else
		return gSensitivityMap[eepromShadowSensitivity];
}

uint8_t menuGetAcceleration(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideAcceleration;
	else
		return eepromShadowAcceleration;
}

uint8_t menuGetDeceleration(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideDeceleration;
	else
		return eepromShadowDeceleration;
}

uint8_t menuGetOuterDeadBand(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideOuterDeadBand;
	else
		return eepromShadowOuterDeadBand;
}

uint8_t menuGetCenterDeadBand(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideCenterDeadBand;
	else
		return eepromShadowCenterDeadBand;
}

uint8_t menuGetPropAsSwitch(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverridePropAsSwitch;
	else
		return eepromShadowPropAsSwitch;
}

uint8_t menuGetInvert(uint8_t overridden)
{
	if (overridden)
		return eepromShadowOverrideInvert;
	else
		return eepromShadowInvert;
}

uint8_t isValidChar(char c)
{
	uint8_t isValid = 0;
	if (c == ' ') {
		isValid = 1;
	}
	if (c >= '0' && c <= '9') {
		isValid = 2;
	}
	if (c >= 'A' && c <= 'Z') {
		isValid = 3;
	}
	if (c >= 'a' && c <= 'z') {
		isValid = 4;
	}
	return isValid;
}

void menuUpdate(int16_t speed, int16_t dir)
{
	uint8_t up = lcdUpFallingEdge();
	uint8_t down = lcdDownFallingEdge();
	uint8_t right = lcdRightFallingEdge();
	uint8_t left = lcdLeftFallingEdge();
	uint8_t leftLongPress = lcdLeftLongPress();
	static uint8_t cursorPosition = 0;

	char lcdLine1[LCD_NUM_CHARACTERS+1];
	char lcdLine2[LCD_NUM_CHARACTERS+1];
	lcdLine1[LCD_NUM_CHARACTERS] = '\0';
	lcdLine2[LCD_NUM_CHARACTERS] = '\0';

	if (eepromShadowIsPlatformDown) {
		sprintf(lcdLine1, "Platform down");
		lcdLine2[0] = '\0';
		lcdText(lcdLine1, lcdLine2, 0);
		gMotorsDisabled = 1;
		return;
	}

	if (eepromShadowMenuState == MENU_OPTION_PROFILE && leftLongPress) {
		gNameEditMode = !gNameEditMode;

		if (gNameEditMode) {
			sprintf(lcdLine1, "Edit name");
			sprintf(lcdLine2, "%s", currentProfileName);
			lcdText(lcdLine1, lcdLine2, 1);

			// Display on, LCD cursor on, blink off
			lcdCommandBlocking(LCD_CMD_DISPLAY_ON_OFF | LCD_CMD_DISPLAY_ON_OFF_D_bm | LCD_CMD_DISPLAY_ON_OFF_C_bm);
			lcdCommandBlocking(LCD_CMD_SET_DDRAM_ADDR | (LCD_DDRAM_ADDR_bm & LCD_LINE_2_START_ADDR));
			cursorPosition = 0;
		} else {
			// Display on, LCD cursor off, blink off
			lcdCommandBlocking(LCD_CMD_DISPLAY_ON_OFF | LCD_CMD_DISPLAY_ON_OFF_D_bm);

			// Write to eeprom
			eepromUpdateStringSafe(currentProfileName, eepromProfileName[eepromShadowCurrentProfile]);
		}
	}

	if (gNameEditMode) {
		gMotorsDisabled = 1;
		if (!isValidChar(currentProfileName[cursorPosition])) {
			currentProfileName[cursorPosition] = ' ';
			currentProfileName[cursorPosition+1] = '\0';
		}
		if (left || right) {
			if (left && cursorPosition > 0) {
				cursorPosition--;
			}
			if (right && cursorPosition < LCD_NUM_CHARACTERS - 1) {
				cursorPosition++;
				if (!isValidChar(currentProfileName[cursorPosition])) {
					currentProfileName[cursorPosition] = ' ';
					currentProfileName[cursorPosition+1] = '\0';
				}
			}
			// Change cursor position
			lcdCommandBlocking((LCD_CMD_SET_DDRAM_ADDR | (LCD_DDRAM_ADDR_bm & LCD_LINE_2_START_ADDR)) + cursorPosition);
		}
		if (up || down) {
			// Change letter
			if (up) {
				if (currentProfileName[cursorPosition] == ' ') {
					currentProfileName[cursorPosition] = 'A';
				} else if (currentProfileName[cursorPosition] == 'Z') {
					currentProfileName[cursorPosition] = 'a';
				} else if (currentProfileName[cursorPosition] == 'z') {
					currentProfileName[cursorPosition] = '0';
				} else if (currentProfileName[cursorPosition] == '9') {
					currentProfileName[cursorPosition] = ' ';
				} else {
					currentProfileName[cursorPosition]++;
				}
			}
			if (down) {
				if (currentProfileName[cursorPosition] == ' ') {
					currentProfileName[cursorPosition] = '9';
				} else if (currentProfileName[cursorPosition] == '0') {
					currentProfileName[cursorPosition] = 'z';
				} else if (currentProfileName[cursorPosition] == 'a') {
					currentProfileName[cursorPosition] = 'Z';
				} else if (currentProfileName[cursorPosition] == 'A') {
					currentProfileName[cursorPosition] = ' ';
				} else {
					currentProfileName[cursorPosition]--;
				}
			}
			sprintf(lcdLine1, "Edit name");
			sprintf(lcdLine2, "%s", currentProfileName);
			lcdText(lcdLine1, lcdLine2, 1);
			lcdCommandBlocking((LCD_CMD_SET_DDRAM_ADDR | (LCD_DDRAM_ADDR_bm & LCD_LINE_2_START_ADDR)) + cursorPosition);
		}
		return;
	}

	gMotorsDisabled = 0;

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
			menuInit();
		}
		if (down && eepromShadowCurrentProfile > 0) {
			eepromUpdateByteSafe(&eepromCurrentProfile, &eepromShadowCurrentProfile, eepromShadowCurrentProfile - 1);
			menuInit();
		}
		sprintf(lcdLine1, "Choose Profile");
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_FWD_THROW:
		if (up && eepromShadowFwdThrow < 2.45) {
			eepromUpdateFloatSafe(&eepromFwdThrow[eepromShadowCurrentProfile], &eepromShadowFwdThrow, eepromShadowFwdThrow + 0.05);
		}
		if (down && eepromShadowFwdThrow > 0.05) {
			eepromUpdateFloatSafe(&eepromFwdThrow[eepromShadowCurrentProfile], &eepromShadowFwdThrow, eepromShadowFwdThrow - 0.05);
		}
		sprintf(lcdLine1, "Fwd Throw: %.2f", (double)eepromShadowFwdThrow);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_REV_THROW:
		if (up && eepromShadowRevThrow < 2.45) {
			eepromUpdateFloatSafe(&eepromRevThrow[eepromShadowCurrentProfile], &eepromShadowRevThrow, eepromShadowRevThrow + 0.05);
		}
		if (down && eepromShadowRevThrow > 0.05) {
			eepromUpdateFloatSafe(&eepromRevThrow[eepromShadowCurrentProfile], &eepromShadowRevThrow, eepromShadowRevThrow - 0.05);
		}
		sprintf(lcdLine1, "Rev Throw: %.2f", (double)eepromShadowRevThrow);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_TURN_THROW:
		if (up && eepromShadowTurnThrow < 2.45) {
			eepromUpdateFloatSafe(&eepromTurnThrow[eepromShadowCurrentProfile], &eepromShadowTurnThrow, eepromShadowTurnThrow + 0.05);
		}
		if (down && eepromShadowTurnThrow > 0.05) {
			eepromUpdateFloatSafe(&eepromTurnThrow[eepromShadowCurrentProfile], &eepromShadowTurnThrow, eepromShadowTurnThrow - 0.05);
		}
		sprintf(lcdLine1, "Turn Throw: %.2f", (double)eepromShadowTurnThrow);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_TOP_FWD_SPEED:
		if (up && eepromShadowTopFwdSpeed < 125) {
			eepromUpdateByteSafe(&eepromTopFwdSpeed[eepromShadowCurrentProfile], &eepromShadowTopFwdSpeed, eepromShadowTopFwdSpeed + 5);
		}
		if (down && eepromShadowTopFwdSpeed > 5) {
			eepromUpdateByteSafe(&eepromTopFwdSpeed[eepromShadowCurrentProfile], &eepromShadowTopFwdSpeed, eepromShadowTopFwdSpeed - 5);
		}
		sprintf(lcdLine1, "Fwd Speed: %d", eepromShadowTopFwdSpeed);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_TOP_REV_SPEED:
		if (up && eepromShadowTopRevSpeed < 125) {
			eepromUpdateByteSafe(&eepromTopRevSpeed[eepromShadowCurrentProfile], &eepromShadowTopRevSpeed, eepromShadowTopRevSpeed + 5);
		}
		if (down && eepromShadowTopRevSpeed > 5) {
			eepromUpdateByteSafe(&eepromTopRevSpeed[eepromShadowCurrentProfile], &eepromShadowTopRevSpeed, eepromShadowTopRevSpeed - 5);
		}
		sprintf(lcdLine1, "Rev Speed: %d", eepromShadowTopRevSpeed);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_TOP_TURN_SPEED:
		if (up && eepromShadowTopTurnSpeed < 125) {
			eepromUpdateByteSafe(&eepromTopTurnSpeed[eepromShadowCurrentProfile], &eepromShadowTopTurnSpeed, eepromShadowTopTurnSpeed + 5);
		}
		if (down && eepromShadowTopTurnSpeed > 5) {
			eepromUpdateByteSafe(&eepromTopTurnSpeed[eepromShadowCurrentProfile], &eepromShadowTopTurnSpeed, eepromShadowTopTurnSpeed - 5);
		}
		sprintf(lcdLine1, "Turn Speed: %d", eepromShadowTopTurnSpeed);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_SENSITIVITY:
		if (up && eepromShadowSensitivity < 9) {
			eepromUpdateByteSafe(&eepromSensitivity[eepromShadowCurrentProfile], &eepromShadowSensitivity, eepromShadowSensitivity + 1);
		}
		if (down && eepromShadowSensitivity > 0) {
			eepromUpdateByteSafe(&eepromSensitivity[eepromShadowCurrentProfile], &eepromShadowSensitivity, eepromShadowSensitivity - 1);
		}
		sprintf(lcdLine1, "Sensitivity: %d", eepromShadowSensitivity + 1);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_ACCELERATION:
		if (up && eepromShadowAcceleration > 4) {
			eepromUpdateByteSafe(&eepromAcceleration[eepromShadowCurrentProfile], &eepromShadowAcceleration, eepromShadowAcceleration - 4);
		}
		if (down && eepromShadowAcceleration < 100) {
			eepromUpdateByteSafe(&eepromAcceleration[eepromShadowCurrentProfile], &eepromShadowAcceleration, eepromShadowAcceleration + 4);
		}
		sprintf(lcdLine1, "Acceleration: %d", (104 - eepromShadowAcceleration) / 4);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_DECELERATION:
		if (up && eepromShadowDeceleration > 4) {
			eepromUpdateByteSafe(&eepromDeceleration[eepromShadowCurrentProfile], &eepromShadowDeceleration, eepromShadowDeceleration - 4);
		}
		if (down && eepromShadowDeceleration < 100) {
			eepromUpdateByteSafe(&eepromDeceleration[eepromShadowCurrentProfile], &eepromShadowDeceleration, eepromShadowDeceleration + 4);
		}
		sprintf(lcdLine1, "Deceleration: %d", (104 - eepromShadowDeceleration) / 4);
		sprintf(lcdLine2, "%s", currentProfileName);
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
			sprintf(lcdLine1, "Outer DB: Off");
		} else if (eepromShadowOuterDeadBand == 1) {
			// 1: immediate
			sprintf(lcdLine1, "Outer DB: Immed.");
		} else {
			// 2: 0.5s, 3: 1.0s, 4: 1.5s, etc
			// Conversion: y=(x-1)/2
			sprintf(lcdLine1, "Outer DB: %d.%ds", (eepromShadowOuterDeadBand-1)/2, (eepromShadowOuterDeadBand-1) % 2 ? 5 : 0);
		}
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_CTR_DEAD_BAND:
		if (up && eepromShadowCenterDeadBand < 56) {
			eepromUpdateByteSafe(&eepromCenterDeadBand[eepromShadowCurrentProfile], &eepromShadowCenterDeadBand, eepromShadowCenterDeadBand + 6);
		}
		if (down && eepromShadowCenterDeadBand > 2) {
			eepromUpdateByteSafe(&eepromCenterDeadBand[eepromShadowCurrentProfile], &eepromShadowCenterDeadBand, eepromShadowCenterDeadBand - 6);
		}
		sprintf(lcdLine1, "Center DB: %d", (eepromShadowCenterDeadBand - 2) / 6 + 1);
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_PROP_AS_SWITCH:
		if (up || down) {
			if (eepromShadowPropAsSwitch) {
				eepromUpdateByteSafe(&eepromPropAsSwitch[eepromShadowCurrentProfile], &eepromShadowPropAsSwitch, 0);
			} else {
				eepromUpdateByteSafe(&eepromPropAsSwitch[eepromShadowCurrentProfile], &eepromShadowPropAsSwitch, 1);
			}
		}
		sprintf(lcdLine1, "PropAsSwitch:%s", eepromShadowPropAsSwitch ? " On" : "Off");
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	case MENU_OPTION_INVERT:
		if (up || down) {
			if (eepromShadowInvert) {
				eepromUpdateByteSafe(&eepromInvert[eepromShadowCurrentProfile], &eepromShadowInvert, 0);
			} else {
				eepromUpdateByteSafe(&eepromInvert[eepromShadowCurrentProfile], &eepromShadowInvert, 1);
			}
		}
		sprintf(lcdLine1, "Invert: %s", eepromShadowInvert ? "On" : "Off");
		sprintf(lcdLine2, "%s", currentProfileName);
		break;
	default:
		lcdLine1[0] = '\0';
		lcdLine2[0] = '\0';
		break;
	}

	if ((up || down) && eepromShadowCurrentProfile == PROFILE_COUNT - 1) {
		menuInit();
	}

	//if (eepromShadowMenuState != MENU_OPTION_PROFILE) {
		//sprintf(lcdLine2, "S=%4d T=%4d%3d", speed, dir, gWirelessTimeoutCount);
	//}

	lcdText(lcdLine1, lcdLine2, 0);
}
