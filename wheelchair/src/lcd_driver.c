/*
 * lcd_driver.c
 *
 * Created: 6/4/2012 4:22:20 PM
 *  Author: Stew
 *
 * Interrupt-driven 4-bit parallel LCD driver for a 2-line LCD
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <stdio.h>
#include <string.h>
#include "lcd_driver.h"

// 0.25 µs per tick
#define LCD_TIMER_CLKSEL TC_CLKSEL_DIV8_gc
#define LCD_NUM_CHARACTERS 16

static volatile uint8_t gLCDState;
static volatile uint8_t gLCDCharPosition;
static volatile char *gLCDCurrentLine;

static volatile char gLCDLine1[LCD_NUM_CHARACTERS + 1];
static volatile char gLCDLine2[LCD_NUM_CHARACTERS + 1];

static inline uint8_t lcdBusy(void)
{
	return gLCDState;
}

static inline void setTimerPeriod(uint16_t period)
{
	TCE1.CCA = period;
	TCE1.CTRLFSET = TC_CMD_RESTART_gc;
}

static inline void stopTimer()
{
	TCE1.CTRLA = TC_CLKSEL_OFF_gc;
	TCE1.CNT = 0;
}

static inline void startTimer()
{
	TCE1.CTRLFSET = TC_CMD_RESTART_gc;
	TCE1.CTRLA = LCD_TIMER_CLKSEL;
}

static inline void lcdESet(void)
{
	LCD_E_PORT.OUTSET = LCD_E_PIN_bm;
}

static inline void lcdEClr(void)
{
	LCD_E_PORT.OUTCLR = LCD_E_PIN_bm;
}

static inline void lcdRSSet(void)
{
	LCD_RS_PORT.OUTSET = LCD_RS_PIN_bm;
}

static inline void lcdRSClr(void)
{
	LCD_RS_PORT.OUTCLR = LCD_RS_PIN_bm;
}

static void lcdSetPortData(uint8_t data) {
	if (data & 0x8) {
		LCD_DB7_PORT.OUTSET = LCD_DB7_PIN_bm;
	} else {
		LCD_DB7_PORT.OUTCLR = LCD_DB7_PIN_bm;
	}
	if (data & 0x4) {
		LCD_DB6_PORT.OUTSET = LCD_DB6_PIN_bm;
	} else {
		LCD_DB6_PORT.OUTCLR = LCD_DB6_PIN_bm;
	}
	if (data & 0x2) {
		LCD_DB5_PORT.OUTSET = LCD_DB5_PIN_bm;
	} else {
		LCD_DB5_PORT.OUTCLR = LCD_DB5_PIN_bm;
	}
	if (data & 0x1) {
		LCD_DB4_PORT.OUTSET = LCD_DB4_PIN_bm;
	} else {
		LCD_DB4_PORT.OUTCLR = LCD_DB4_PIN_bm;
	}
}

static void lcdNibble(uint8_t data)
{
	lcdSetPortData(data);
	lcdESet();
	_delay_us(1);
	lcdEClr();
}

static void lcdCommand(uint8_t command)
{
	lcdRSClr();
	lcdNibble(command >> 4);
	_delay_us(1);
	lcdNibble(command & 0x0F);

	if ((command == LCD_CMD_CLEAR_DISPLAY) || ((command & 0xFE) == LCD_CMD_RETURN_HOME)) {
		_delay_ms(1.52);
	} else {
		_delay_us(38);
	}
}

static inline void lcdStartWrite(void)
{
	// case 0: Set E - ADDRLINE1[7:4]
	lcdRSClr();
	lcdESet();
	lcdSetPortData((LCD_CMD_SET_DDRAM_ADDR | (LCD_DDRAM_ADDR_bm & LCD_LINE_1_START_ADDR)) >> 4);
	gLCDState = 1;
	gLCDCurrentLine = gLCDLine1;
	setTimerPeriod(4);
	startTimer();
}

// Copies src into dest, max LCD_NUM_CHARACTERS chars, pads with ' ', null terminates
static void copyString(volatile char *dest, const char *src)
{
	size_t i;
	size_t srclen = strlen(src);
	if (srclen > LCD_NUM_CHARACTERS) {
		srclen = LCD_NUM_CHARACTERS;
	}
	for (i = 0; i < srclen; i++) {
		dest[i] = src[i];
	}
	for (; i < LCD_NUM_CHARACTERS; i++) {
		dest[i] = ' ';
	}
	dest[i] = '\0';
}

void lcdText(const char *line1, const char *line2, uint8_t blocking)
{
	if (blocking) {
		while (lcdBusy()) {
			// twiddle thumbs
		}
	} else if (lcdBusy()) {
		return;
	}

	copyString(gLCDLine1, line1);
	copyString(gLCDLine2, line2);

	lcdStartWrite();
}

void initLCDDriver(void)
{
	// I/O pin setup
	LCD_DB7_PORT.DIRSET = LCD_DB7_PIN_bm;
	LCD_DB7_PORT.OUTCLR = LCD_DB7_PIN_bm;
	LCD_DB6_PORT.DIRSET = LCD_DB6_PIN_bm;
	LCD_DB6_PORT.OUTCLR = LCD_DB6_PIN_bm;
	LCD_DB5_PORT.DIRSET = LCD_DB5_PIN_bm;
	LCD_DB5_PORT.OUTCLR = LCD_DB5_PIN_bm;
	LCD_DB4_PORT.DIRSET = LCD_DB4_PIN_bm;
	LCD_DB4_PORT.OUTCLR = LCD_DB4_PIN_bm;
	LCD_RS_PORT.DIRSET = LCD_RS_PIN_bm;
	LCD_RS_PORT.OUTCLR = LCD_RS_PIN_bm;
	LCD_RW_PORT.DIRSET = LCD_RW_PIN_bm;
	LCD_RW_PORT.OUTCLR = LCD_RW_PIN_bm;
	LCD_E_PORT.DIRSET = LCD_E_PIN_bm;
	LCD_E_PORT.OUTCLR = LCD_E_PIN_bm;

	// Timer setup
	TCE1.CTRLB = TC_WGMODE_FRQ_gc;
	TCE1.INTCTRLB = TC_CCAINTLVL_LO_gc;
	gLCDState = 0;

	// LCD init sequence

	_delay_ms(40); //Wait >40 msec after power is applied

	// Start out as 8-bit bus

	lcdRSClr();
	lcdNibble((LCD_CMD_FUNCTION_SET | LCD_CMD_FUNCTION_DL_bm) >> 4); //Wake up #1
	_delay_ms(5);
	lcdNibble((LCD_CMD_FUNCTION_SET | LCD_CMD_FUNCTION_DL_bm) >> 4); //Wake up #2
	_delay_us(160);
	lcdNibble((LCD_CMD_FUNCTION_SET | LCD_CMD_FUNCTION_DL_bm) >> 4); //Wake up #3
	_delay_us(160);
	lcdNibble((LCD_CMD_FUNCTION_SET) >> 4); // Tell the LCD to switch to 4-bit bus (this command is still 8-bit)
	_delay_us(38);

	//Now it's a 4-bit bus

	lcdCommand(LCD_CMD_FUNCTION_SET | LCD_CMD_FUNCTION_N_bm); // 2-line LCD, 5x8
	lcdCommand(LCD_CMD_DISPLAY_ON_OFF | LCD_CMD_DISPLAY_ON_OFF_D_bm); //Display ON
	lcdCommand(LCD_CMD_ENTRY_MODE_SET | LCD_CMD_ENTRY_MODE_I_D_bm); //Cursor moves right
	lcdCommand(LCD_CMD_CLEAR_DISPLAY); // Last thing to do before writing text
}

ISR(TCE1_CCA_vect)
{
	switch (gLCDState)
	{
	default:
		while (1)
		{
			// Error - wait for watchdog reset
		}
		break;
	case 1: // Clear E - ADDRLINEx[7:4]
		lcdEClr();
		gLCDState = 2;
		break;
	case 2: // Set E - ADDRLINEx[3:0]
		lcdESet();
		if (gLCDCurrentLine == gLCDLine1) {
			lcdSetPortData((LCD_CMD_SET_DDRAM_ADDR | (LCD_DDRAM_ADDR_bm & LCD_LINE_1_START_ADDR)) & 0x0F);
		} else {
			lcdSetPortData((LCD_CMD_SET_DDRAM_ADDR | (LCD_DDRAM_ADDR_bm & LCD_LINE_2_START_ADDR)) & 0x0F);
		}
		gLCDState = 3;
		break;
	case 3: // Clear E - ADDRLINEx[3:0]
		lcdEClr();
		gLCDCharPosition = 0;
		gLCDState = 4;
		setTimerPeriod(152);
		break;
	case 4: // Set E - CHAR[7:4]
		lcdRSSet();
		lcdESet();
		lcdSetPortData(gLCDCurrentLine[gLCDCharPosition] >> 4);
		gLCDState = 5;
		setTimerPeriod(4);
		break;
	case 5: // Clear E - CHAR[7:4]
		lcdEClr();
		gLCDState = 6;
		break;
	case 6: // Set E - CHAR[3:0]
		lcdESet();
		lcdSetPortData(gLCDCurrentLine[gLCDCharPosition] & 0x0F);
		gLCDState = 7;
		break;
	case 7: // Clear E - CHAR[3:0]
		lcdEClr();
		gLCDCharPosition++;
		if (gLCDCharPosition >= LCD_NUM_CHARACTERS) {
			if (gLCDCurrentLine == gLCDLine1) {
				gLCDState = 8;
			} else {
				gLCDState = 9;
			}
		} else {
			gLCDState = 4;
		}
		setTimerPeriod(152);
		break;
	case 8: // Set E - ADDRLINE2[7:4]
		lcdRSClr();
		lcdESet();
		lcdSetPortData((LCD_CMD_SET_DDRAM_ADDR | (LCD_DDRAM_ADDR_bm & LCD_LINE_2_START_ADDR)) >> 4);
		gLCDState = 1;
		gLCDCurrentLine = gLCDLine2;
		setTimerPeriod(4);
		break;
	case 9: // End
		stopTimer();
		gLCDState = 0;
		break;
	}
}
