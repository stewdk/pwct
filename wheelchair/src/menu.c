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

float EEMEM eepromThrow = 0.7;

void menuUpdate(int16_t speed, int16_t dir)
{
	if (lcdUpFallingEdge())
	{
		eeprom_update_float(&eepromThrow, eeprom_read_float(&eepromThrow) + 0.05);
		printf("EEPROM written\n");
	}
	if (lcdDownFallingEdge())
	{
		eeprom_update_float(&eepromThrow, eeprom_read_float(&eepromThrow) - 0.05);
		printf("EEPROM written\n");
	}
	if (lcdRightFallingEdge())
	{
		//printf("Right\n");
		//lcdText("Right", "", 0);
	}
	if (lcdLeftFallingEdge())
	{
		//printf("Left\n");
		//lcdText("Left", "", 0);
	}

	//printf("Speed: %d Dir: %d\n", speed, dir);
	char lcdLine1[17];
	char lcdLine2[17];
	lcdLine1[16] = '\0';
	lcdLine2[16] = '\0';

	//sprintf(lcdLine1, "Speed: %d", speed);
	//sprintf(lcdLine2, " Turn: %d", dir);

	sprintf(lcdLine1, "Throw=%.2f", (double)menuGetThrow());
	sprintf(lcdLine2, "S=%4d T=%4d", speed, dir);

	lcdText(lcdLine1, lcdLine2, 0);
}

float menuGetThrow(void)
{
	return eeprom_read_float(&eepromThrow);
}
