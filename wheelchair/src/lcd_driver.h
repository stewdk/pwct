/*
 * lcd_driver.h
 *
 * Created: 6/4/2012 4:22:33 PM
 *  Author: Stew
 */ 

#ifndef LCD_DRIVER_H_
#define LCD_DRIVER_H_

// BEGIN HARDWARE SPECIFIC

// DB7 = PC7
#define LCD_DB7_PORT PORTC
#define LCD_DB7_PIN_bm PIN7_bm

// DB6 = PC6
#define LCD_DB6_PORT PORTC
#define LCD_DB6_PIN_bm PIN6_bm

// DB5 = PC5
#define LCD_DB5_PORT PORTC
#define LCD_DB5_PIN_bm PIN5_bm

// DB4 = PC4
#define LCD_DB4_PORT PORTC
#define LCD_DB4_PIN_bm PIN4_bm

// Operation Enable, E = PD1
#define LCD_E_PORT PORTD
#define LCD_E_PIN_bm PIN1_bm

// Read/Write, RW = PD4
#define LCD_RW_PORT PORTD
#define LCD_RW_PIN_bm PIN4_bm
//RW=0: write
//RW=1: read WARNING READ NOT SUPPORTED BY HARDWARE

// Register Select, RS = PD5
#define LCD_RS_PORT PORTD
#define LCD_RS_PIN_bm PIN5_bm
//RS=0: instruction
//RS=1: data

// END HARDWARE SPECIFIC


// Instructions/Commands

#define LCD_CMD_CLEAR_DISPLAY 0x01
	// Execution time = 1.52msec
	// Also returns home

#define LCD_CMD_RETURN_HOME 0x02
	// Execution time = 1.52msec

#define LCD_CMD_ENTRY_MODE_SET 0x04
	// Execution time = 38usec
#define LCD_CMD_ENTRY_MODE_I_D_bm 0x02
	// I/D=1: cursor moves right / DDRAM address increments / display shift left
	// I/D=0: cursor moves left / DDRAM address decrements / display shift right
#define LCD_CMD_ENTRY_MODE_S_bm 0x01
	// S=0: don't shift display
	// S=1: during DDRAM write operation, shift display

#define LCD_CMD_DISPLAY_ON_OFF 0x08
	// Execution time = 38usec
#define LCD_CMD_DISPLAY_ON_OFF_D_bm 0x04
	// D=1: display on
	// D=0: display off
#define LCD_CMD_DISPLAY_ON_OFF_C_bm 0x02
	// C=1: cursor on
	// C=0: cursor off
#define LCD_CMD_DISPLAY_ON_OFF_B_bm 0x01
	// B=1: cursor blink on
	// B=0: cursor blink off

#define LCD_CMD_CURSOR_OR_DISPLAY_SHIFT 0x10
	// Execution time = 38usec
#define LCD_CMD_CURSOR_OR_DISPLAY_SHIFT_S_C_bm 0x08
	// S/C=0: shift cursor
	// S/C=1: shift display
#define LCD_CMD_CURSOR_OR_DISPLAY_SHIFT_R_L_bm 0x04
	// R/L=0: shift left
	// R/L=1: shift right

#define LCD_CMD_FUNCTION_SET 0x20
	// Execution time = 38usec
#define LCD_CMD_FUNCTION_DL_bm 0x10
	// DL=1: 8 bit bus
	// DL=0: 4 bit bus
#define LCD_CMD_FUNCTION_N_bm 0x08
	// N=0: 1-line display
	// N=1: 2-line display
#define LCD_CMD_FUNCTION_F_bm 0x04
	// F=0: 5x8
	// F=1: 5x11

#define LCD_CMD_SET_CGRAM_ADDR 0x40
	// Execution time = 38usec
#define LCD_CGRAM_ADDR_bm 0x3F

#define LCD_CMD_SET_DDRAM_ADDR 0x80
	// Execution time = 38usec
#define LCD_DDRAM_ADDR_bm 0x7F

#define LCD_LINE_1_START_ADDR 0x00
#define LCD_LINE_2_START_ADDR 0x40

void initLCDDriver(void);
void lcdText(const char *line1, const char *line2);

#endif /* LCD_DRIVER_H_ */
