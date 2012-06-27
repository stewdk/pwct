/*
 * lcd_driver.c
 *
 * Created: 6/4/2012 4:22:20 PM
 *  Author: Stew
 */ 

#include "../atmel/avr_compiler.h"
#include <avr/io.h>
#include <string.h>
#include "lcd_driver.h"

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

static void lcdNibble(uint8_t data)
{
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

static void lcdWrite(uint8_t data)
{
	lcdRSSet();
	lcdNibble(data >> 4);
	_delay_us(1);
	lcdNibble(data & 0x0F);
	_delay_us(38);
}

static inline void lcdSetAddr(uint8_t addr)
{
	lcdCommand(LCD_CMD_SET_DDRAM_ADDR | (LCD_DDRAM_ADDR_bm & addr));
}

static void lcdLine(const char *text, uint8_t startAddr) {
	size_t i;
	size_t textlen = strlen(text);
	if (textlen > 16) {
		textlen = 16;
	}
	lcdSetAddr(startAddr);
	for (i = 0; i < textlen; i++) {
		lcdWrite(text[i]);
	}
}

void lcdText(const char *line1, const char *line2)
{
	lcdLine(line1, LCD_LINE_1_START_ADDR);
	lcdLine(line2, LCD_LINE_2_START_ADDR);
}

void initLCDDriver(void)
{
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
	lcdCommand(LCD_CMD_DISPLAY_ON_OFF | LCD_CMD_DISPLAY_ON_OFF_D_bm | LCD_CMD_DISPLAY_ON_OFF_C_bm); //Display ON
	lcdCommand(LCD_CMD_ENTRY_MODE_SET | LCD_CMD_ENTRY_MODE_I_D_bm); //Cursor moves right
	lcdCommand(LCD_CMD_CLEAR_DISPLAY); // Last thing to do before writing text
}
