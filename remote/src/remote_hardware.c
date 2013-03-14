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

#ifdef INSTRUCTOR_REMOTE
#define DEBOUNCED_INPUT_COUNT 1
static debounced_input gDebouncedInputs[DEBOUNCED_INPUT_COUNT];

static void setupDebouncedInputs()
{
	// Assumption: pins are already set as input and pull-ups enabled

	//E-stop button
	#define DEBOUNCE_INDEX_ESTOP 0
	gDebouncedInputs[DEBOUNCE_INDEX_ESTOP].pin = &PINA;
	gDebouncedInputs[DEBOUNCE_INDEX_ESTOP].pin_bm = _BV(PA1);

	int i;
	for (i = 0; i < DEBOUNCED_INPUT_COUNT; i++)
	{
		// Default values
		gDebouncedInputs[i].previous_values = UINT8_MAX;
		gDebouncedInputs[i].debounced_value = 1;
	}
}
#endif // INSTRUCTOR_REMOTE

static void setupDebounceTimer()
{
	//TCCR0A: TCW0 ICEN0 ICNC0 ICES0 ACIC0 - - CTC0
	TCCR0A = _BV(CTC0);

	// Prescaler = 64
	//TCCR0B: - - - TSM PSR0 CS02 CS01 CS00
	TCCR0B = _BV(CS01) | _BV(CS00);

	// Goal: interrupt every 5 milliseconds
	OCR0A = 78;

	//TIMSK: OCIE1D OCIE1A OCIE1B OCIE0A OCIE0B TOIE1 TOIE0 TICIE0
	TIMSK |= _BV(OCIE0A);
}

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

#ifdef INSTRUCTOR_REMOTE
	setupDebouncedInputs();
#endif // INSTRUCTOR_REMOTE
	setupDebounceTimer();

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

// Turns the LED on only after a 0.25 delay
static volatile uint8_t gLEDDelayOn = 0;
void setLEDDelay()
{
	gLEDDelayOn = 1;
}

// Turns the LED off immediately and resets the LED delay
void clrLEDDelay()
{
	gLEDDelayOn = 0;
}

uint8_t getADC5(void)
{
	return valueADC5;	// Left/Right
}

uint8_t getADC6(void)
{
	return valueADC6;	// Up/Down
}

#ifdef INSTRUCTOR_REMOTE
uint8_t getEStop(void)
{
	return !gDebouncedInputs[DEBOUNCE_INDEX_ESTOP].debounced_value;
}
#endif // INSTRUCTOR_REMOTE

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

// Debounce timer ISR
ISR(TIMER0_COMPA_vect)
{
	static uint8_t ledOnCount = 0;
#ifdef INSTRUCTOR_REMOTE
	int i;
	for (i = 0; i < DEBOUNCED_INPUT_COUNT; i++)
	{
		gDebouncedInputs[i].previous_values = (gDebouncedInputs[i].previous_values << 1) | ((*(gDebouncedInputs[i].pin) & gDebouncedInputs[i].pin_bm) ? 1 : 0);

		if (gDebouncedInputs[i].previous_values == UINT8_MAX)
		{
			gDebouncedInputs[i].debounced_value = 1;
		}
		else if (gDebouncedInputs[i].previous_values == 0)
		{
			gDebouncedInputs[i].debounced_value = 0;
		}
	}
#endif // INSTRUCTOR_REMOTE

	if (gLEDDelayOn) {
		if (ledOnCount < 50) {
			ledOnCount++;
		} else {
			setLED();
		}
	} else {
		clrLED();
		ledOnCount = 0;
	}
}
