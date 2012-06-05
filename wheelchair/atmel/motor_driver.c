/*
 * motor_driver.c
 *
 * Created: 6/4/2012 3:28:23 PM
 *  Author: Stew
 */ 

#include <avr/io.h>

/* S1 = PD7 (TXD1, USARTD1)
 * S2 = PD6 (RXD1, USARTD1)
 */

void initMotorDriver(void) {
	PORTD.DIRSET = PIN6_bm | PIN7_bm;
	PORTD.OUTSET = PIN6_bm | PIN7_bm;
}
