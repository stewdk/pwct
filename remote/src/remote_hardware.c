/*
 * remote_hardware.c
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "remote_hardware.h"

static uint8_t valueADC5;
static uint8_t valueADC6;

void initHardware(void)
{
//Init LED
	PORTB &= ~_BV(PB3);
	DDRB |= _BV(PB3);

//Init ADC values
	valueADC5 = 125;
	valueADC6 = 125;

	//set buttons pins as inputs
	DDRA &= ~(_BV(PA0) | _BV(PA1) | _BV(PA2) | _BV(PA3) | _BV(PA4) | _BV(PA5));
	//enable pullups on button lines
	PORTA |= _BV(PA0) | _BV(PA1) | _BV(PA2) | _BV(PA3) | _BV(PA4) | _BV(PA5);

//INIT ADC
	// ADMUX = REFS1:0 ADLAR MUX4:0
	// Select Vcc voltage reference, left adjust result
#define ADMUX_ADC5_bm (_BV(ADLAR) | 0x05) //ADC5 = PA6 = x-axis = horizontal axis = right/left = joystick direction = blue wire
#define ADMUX_ADC6_bm (_BV(ADLAR) | 0x06) //ADC6 = PA7 = y-axis = vertical axis = fwd/rev = joystick speed = yellow wire
	ADMUX = ADMUX_ADC5_bm;

	// ADCSRA = ADEN ADSC ADATE ADIF ADIE ADPS2:0
	ADCSRA = _BV(ADEN) | _BV(ADIE) | _BV(ADPS2); //Enable ADC, interrupt enabled, clock divider  16 (62 KHz@1MHz)

	// ADCSRB = BIN GSEL - REFS2 MUX5 ADTS2:0

	DIDR0 = _BV(ADC5D) | _BV(ADC6D);	//disable digital inputs on channels used for analog

	ADCSRA |= _BV(ADSC); //Start conversion
}

inline void setLED(void)
{
	PORTB |= _BV(PB3);
}

inline void clrLED(void)
{
	PORTB &= ~_BV(PB3);
}

inline void tglLED(void)
{
	if(PORTB & _BV(PB3)) {
		PORTB &= ~_BV(PB3);
	} else {
		PORTB |= _BV(PB3);
	}
}

uint8_t getADC5(void)
{
	return valueADC5;	// Left/Right
}

uint8_t getADC6(void)
{
	return valueADC6;	// Up/Down
}

uint8_t getEStop(void)
{
	if (PINA & _BV(PA0) && PINA & _BV(PA1))
		return 0;
	else
		return 1;
}

#ifdef STUDENT_JOYSTICK
uint8_t isJoystickEnabled()
{
	return !(PINA & _BV(PA1));
}

uint8_t getBuddyButtons()
{
	uint8_t buttonMask = 0;
	if (!(PINA & _BV(PA4))) {
		buttonMask |= 0b00001000; // Forward
	}
	if (!(PINA & _BV(PA2))) {
		buttonMask |= 0b00010000; // Reverse
	}
	if (!(PINA & _BV(PA5))) {
		buttonMask |= 0b00100000; // Left
	}
	if (!(PINA & _BV(PA3))) {
		buttonMask |= 0b01000000; // Right
	}
	return buttonMask;
}
#endif // STUDENT_JOYSTICK

ISR(ADC_vect)
{
	if (ADMUX == ADMUX_ADC5_bm)
	{
		valueADC5 = ADCH;

		ADMUX = ADMUX_ADC6_bm;
	}
	else
	{
		valueADC6 = ADCH;

		ADMUX = ADMUX_ADC5_bm;
	}
	ADCSRA |= _BV(ADSC); //Start conversion
}
