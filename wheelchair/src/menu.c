/*
 * menu.c
 *
 * Created: 8/16/2012 10:19:14 AM
 *  Author: Stew
 */ 

#include <stdio.h>
#include <avr/eeprom.h>
#include "menu.h"
#include "PWCT_io.h"
#include "lcd_driver.h"

// Define the order of the menu options, zero-based
#define MENU_OPTION_THROW 0
#define MENU_OPTION_CTR_DEAD_BAND 1
// We must know how many menu options there are
#define LAST_MENU_OPTION MENU_OPTION_CTR_DEAD_BAND

// EEPROM variables and initial values
// Note: the initial values are only updated when programming the EEPROM memory (wheelchair.eep)
uint8_t EEMEM eepromMenuState = 0;
float EEMEM eepromThrow = 0.8;
uint8_t EEMEM eepromCenterDeadBand = 3;

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
	case MENU_OPTION_THROW:
		if (up && menuGetThrow() < 2.45) {
			eeprom_update_float(&eepromThrow, menuGetThrow() + 0.05);
			printf("EEPROM written\n");
		}
		if (down && menuGetThrow() > 0.05) {
			eeprom_update_float(&eepromThrow, menuGetThrow() - 0.05);
			printf("EEPROM written\n");
		}
		sprintf(lcdLine1, "Throw=%.2f", (double)menuGetThrow());
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
	default:
		lcdLine1[0] = '\0';
		break;
	}
	sprintf(lcdLine2, "S=%4d T=%4d", speed, dir);

	lcdText(lcdLine1, lcdLine2, 0);
}

float menuGetThrow(void)
{
	return eeprom_read_float(&eepromThrow);
}

uint8_t menuGetCenterDeadBand(void)
{
	return eeprom_read_byte(&eepromCenterDeadBand);
}
