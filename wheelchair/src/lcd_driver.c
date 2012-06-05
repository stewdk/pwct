/*
 * lcd_driver.c
 *
 * Created: 6/4/2012 4:22:20 PM
 *  Author: Stew
 */ 

#include <avr/io.h>

/* LCD DB0:7 = PC0:7
 * LCD E = PD1
 * LCD RW = PD4
 * LCD RS = PD5
 */

void initLCDDriver(void) {
	PORTC.DIRSET = 0xFF;
	PORTC.OUTCLR = 0xFF;
	PORTD.DIRSET = PIN1_bm | PIN4_bm | PIN5_bm;
	PORTD.OUTCLR = PIN1_bm | PIN4_bm | PIN5_bm;
}
