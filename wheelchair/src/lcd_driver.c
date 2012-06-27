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

static void lcdCommand(uint8_t command)
{
	LCD_DATA_PORT.OUT = command;
	lcdRSClr();
	lcdESet();
	_delay_us(1);
	lcdEClr();
	if ((command == LCD_CMD_CLEAR_DISPLAY) || ((command & 0xFE) == LCD_CMD_RETURN_HOME)) {
		_delay_ms(1.52);
	} else {
		_delay_us(38);
	}
}

static void lcdWrite(uint8_t data)
{
	LCD_DATA_PORT.OUT = data;
	lcdRSSet();
	lcdESet();
	_delay_us(1);
	lcdEClr();
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
	LCD_DATA_PORT.DIRSET = 0xFF;
	LCD_DATA_PORT.OUTCLR = 0xFF;
	LCD_RS_PORT.DIRSET = LCD_RS_PIN_bm;
	LCD_RS_PORT.OUTCLR = LCD_RS_PIN_bm;
	LCD_RW_PORT.DIRSET = LCD_RW_PIN_bm;
	LCD_RW_PORT.OUTCLR = LCD_RW_PIN_bm;
	LCD_E_PORT.DIRSET = LCD_E_PIN_bm;
	LCD_E_PORT.OUTCLR = LCD_E_PIN_bm;

	lcdEClr();
	_delay_ms(40); //Wait >40 msec after power is applied
	lcdCommand(LCD_CMD_FUNCTION_SET | LCD_CMD_FUNCTION_DL_bm | LCD_CMD_FUNCTION_N_bm); //Wake up #1
	_delay_ms(5);
	lcdCommand(LCD_CMD_FUNCTION_SET | LCD_CMD_FUNCTION_DL_bm | LCD_CMD_FUNCTION_N_bm); //Wake up #2
	_delay_us(160);
	lcdCommand(LCD_CMD_FUNCTION_SET | LCD_CMD_FUNCTION_DL_bm | LCD_CMD_FUNCTION_N_bm); //Wake up #3
	_delay_us(160);
	lcdCommand(LCD_CMD_FUNCTION_SET | LCD_CMD_FUNCTION_DL_bm | LCD_CMD_FUNCTION_N_bm); // 8-bit bus, 2-line LCD
	lcdCommand(LCD_CMD_DISPLAY_ON_OFF | LCD_CMD_DISPLAY_ON_OFF_D_bm); // | LCD_CMD_DISPLAY_ON_OFF_C_bm); //Display ON
	lcdCommand(LCD_CMD_ENTRY_MODE_SET | LCD_CMD_ENTRY_MODE_I_D_bm); //Cursor moves right
	lcdCommand(LCD_CMD_CLEAR_DISPLAY); // Last thing to do before writing text

	lcdText("PWCT", "Ver. 2012-06-27");
}
