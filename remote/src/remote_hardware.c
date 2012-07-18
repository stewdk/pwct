/*
 * remote_hardware.c
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */

#include <avr/io.h>
#include <avr/interrupt.h>
#include "remote_hardware.h"

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

//can't be more than 255
#define msecToWait 100
#define msecTimeout	1000

static uint8_t valueADC5;
static uint8_t valueADC6;

static uint8_t BUTTON_UP;
static uint8_t BUTTON_DOWN;
static uint8_t BUTTON_DISABLE;

static uint8_t DEBOUNCE_DONE;
static uint8_t TIMEOUT;

static uint8_t JOY_STATE;

static uint8_t BUTTON_START_STATE;

static uint8_t BUTTON_CHANGED_FLAG = 0;

void initHardware(void)
{
//Init LED
	cbi(PORTB, PB3);
	sbi(DDRB, PB3);

//Init Buttons
	valueADC5 = 125;
	valueADC6 = 125;
	BUTTON_UP = 0;
	BUTTON_DOWN = 0;
	BUTTON_DISABLE = 0;
	TIMEOUT = 0;
	JOY_STATE = 0;

	//set buttons pins as inputs
	DDRA &= ~(_BV(PA0) | _BV(PA1) | _BV(PA2) | _BV(PA3) | _BV(PA4) | _BV(PA5));
	//enable pullups on button lines
	PORTA |= _BV(PA0) | _BV(PA1) | _BV(PA2) | _BV(PA3) | _BV(PA4) | _BV(PA5);

	//enable button interrupts to wake remote
	//PA0	Button	PCINT0
	//PA1	Up		PCINT1
	//PA2	Down	PCINT2
	PCMSK0 = _BV(PCINT0) | _BV(PCINT1) | _BV(PCINT2) | _BV(PCINT6) | _BV(PCINT7);
	PCMSK1 = 0;
	sbi(GIMSK, PCIE1);

	//set up button debounce
	//start timer /1024
	TCCR1B = _BV(CS13) | _BV(CS11) | _BV(CS10);

	//setup timeout timer
	TCCR0A = _BV(TCW0);	//16 bit mode
	TCNT0H = 0;
	TCNT0L = 0;
	TCCR0B = _BV(CS02) | _BV(CS00);	// clk/1024
	OCR0B = (uint8_t)((msecTimeout & 0xFF00) >> 8);
	OCR0A = (uint8_t)((msecTimeout & 0x00FF) >> 0);
	TIMSK |= _BV(OCIE0A); //compare A interrupt enabled

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
	sbi(PORTB, PB3);
}

inline void clrLED(void)
{
	cbi(PORTB, PB3);
}


inline void tglLED(void)
{
	if(PORTB & _BV(PB3)) {
		cbi(PORTB, PB3);
	} else {
		sbi(PORTB, PB3);
	}
}


uint8_t GetADC5(void)
{
	return valueADC5;	// Left/Right
}

uint8_t GetADC6(void)
{
	return valueADC6;	// Up/Down
}

uint8_t GetButton(void)
{
	return BUTTON_DISABLE | BUTTON_UP | BUTTON_DOWN;
}

uint8_t GetJoyState(void)
{
	return JOY_STATE;
}

uint8_t isDebounceDone(void)
{
	return DEBOUNCE_DONE;
}

uint8_t hasButtonChanged(void)
{
	return BUTTON_CHANGED_FLAG;
}

void clrButtonChanged(void)
{
	BUTTON_CHANGED_FLAG = 0;
}
/*
void clrDebounceFlag(void) {
	DEBOUNCE_DONE = 0;
}
*/
/*
uint8_t noButtonsPressed(void)
{
	return (BUTTON_UP | BUTTON_DOWN | BUTTON_DISABLE) ? 0 : 1;
}
*/
ISR(ADC_vect)
{
	// Todo: this is where we need to fix the up/down, left/right problem (I think)
	if (ADMUX == ADMUX_ADC5_bm)
	{
		if (valueADC5 != ADCH) {
			BUTTON_CHANGED_FLAG = 1;
		}
		valueADC5 = ADCH;

		if(valueADC5 > 170 && (JOY_STATE & 0b01100000) != 0b00100000) {	//left
			JOY_STATE = (JOY_STATE & 0b10011111) | 0b00100000;
			BUTTON_CHANGED_FLAG = 1;
		}
		else if(valueADC5 < 90 && (JOY_STATE & 0b01100000) != 0b01000000) {	//right
			JOY_STATE = (JOY_STATE & 0b10011111) | 0b01000000;
			BUTTON_CHANGED_FLAG = 1;
		}
		else if(valueADC5 >= 90 && valueADC5<=170 && (JOY_STATE & 0b01100000)) {
			JOY_STATE = (JOY_STATE & 0b10011111) | 0b00000000;
			BUTTON_CHANGED_FLAG = 1;
		}
		ADMUX = ADMUX_ADC6_bm;
	}
	else
	{
		if (valueADC6 != ADCH) {
			BUTTON_CHANGED_FLAG = 1;
		}
		valueADC6 = ADCH;

		if(valueADC6 > 170 && (JOY_STATE & 0b00011000) != 0b00010000) {	//down
			JOY_STATE = (JOY_STATE & 0b11100111) | 0b00010000;
			BUTTON_CHANGED_FLAG = 1;
		}
		else if(valueADC6 < 90 && (JOY_STATE & 0b00011000) != 0b00001000) {	//up
			JOY_STATE = (JOY_STATE & 0b11100111) | 0b00001000;
			BUTTON_CHANGED_FLAG = 1;
		}
		else if(valueADC6 >= 90 && valueADC6<=170 && (JOY_STATE & 0b00011000)) {
			JOY_STATE = (JOY_STATE & 0b11100111) | 0b00000000;
			BUTTON_CHANGED_FLAG = 1;
		}
		ADMUX = ADMUX_ADC5_bm;
	}
	ADCSRA |= _BV(ADSC);//Start conversion
}

uint8_t isTimeOut(void)
{
	return TIMEOUT;
}

void resetTimeOut(void)
{
	TIMEOUT = 0;
	TCNT0H = 0;
	TCNT0L = 0;
	TIMSK |= _BV(OCIE0A); //compare A interrupt enabled
}

ISR(PCINT_vect)
{
	uint8_t tempPINA;

	tempPINA = PINA;

	//This vector is here to wake unit up from sleep mode
	DEBOUNCE_DONE =  0;

	//set compare timer
	OCR1A = msecToWait+TCNT1;
	TIMSK |= 0b01000000; //compare A interrupt enabled

	BUTTON_START_STATE = 0;
	BUTTON_START_STATE |= tempPINA & _BV(PA0);
	BUTTON_START_STATE |= tempPINA & _BV(PA1);
	BUTTON_START_STATE |= tempPINA & _BV(PA2);
}

ISR(TIMER1_COMPA_vect)
{
	uint8_t tempPINA;

	tempPINA = PINA;

	DEBOUNCE_DONE = 1;

	TIMSK &= ~0b01000000; //compare A interrupt disabled

	if( (tempPINA & _BV(PA0)) == (BUTTON_START_STATE & _BV(PA0))) {
		BUTTON_DISABLE	= ~tempPINA & _BV(PA0);
		//tglLED();
	}

	if( (tempPINA & _BV(PA1)) == (BUTTON_START_STATE & _BV(PA1))) {
		BUTTON_UP		= ~tempPINA & _BV(PA1);
	}

	if( (tempPINA & _BV(PA2)) == (BUTTON_START_STATE & _BV(PA2))) {
		BUTTON_DOWN		= ~tempPINA & _BV(PA2);
	}

	BUTTON_CHANGED_FLAG = 1;
}

//timeout ISR
ISR(TIMER0_COMPA_vect)
{
	TIMEOUT = 1;
	TIMSK &= ~_BV(OCIE0A); //compare A interrupt disabled
}
