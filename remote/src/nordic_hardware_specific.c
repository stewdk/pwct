/*
 * nordic_hardware_specific.c
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */
#include "nordic_hardware_specific.h"

#define BITBANG_SPI 1

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

#define BOARD_WHEELCHAIR	1
#define BOARD_REMOTE		2

//#define BOARD BOARD_WHEELCHAIR
#define BOARD BOARD_REMOTE

//pull CS low
inline void chipSelect(void)
{
	cbi(PORTB, PB5);
	sbi(DDRB, PB5);
}

//release SS
inline void chipRelease(void)
{
	sbi(PORTB, PB5);
	cbi(DDRB, PB5);
}

//set CE low
inline uint8_t standbyMode(void)
{
	uint8_t mode;
	mode = PORTB & _BV(PB4);
	cbi(PORTB, PB4);
	return mode;
}

//set CE high
inline void activeMode(void)
{
	sbi(PORTB, PB4);
}

//set CE low if mode == 0, else set CE high
inline void setMode(uint8_t mode)
{
	if(mode) {
		activeMode();
	} else {
		standbyMode();
	}
}

//Initialize microcontroller pin directions, spi, interrupts
void initalizeHardwareForNordic(void)
{
	//init SPI DDR
	//MOSI, CLK, CS are outputs
#if BITBANG_SPI
	DDRB |= _BV(PB0) | _BV(PB2) | _BV(PB5);
#endif
	//set chip enable (CE) pin as output, set CE to low for standby mode
	cbi(PORTB, PB4);
	sbi(DDRB, PB4);

	//init IRQ interrupt, PB6, INT0
	sbi(MCUCR, ISC01);
	sbi(GIMSK, INT0);
}

uint8_t SPI_TransceiveByte(uint8_t data)
{
	//Bit Bang SPI
#if BITBANG_SPI
#define TX_PORT		PORTB
#define TX_PORT_PIN	PINB
#define TX_PORT_DD	DDRB
#define TX_SCK	2 //Output
#define TX_MISO 1 //Input
#define TX_MOSI	0 //Output
//#define RF_DELAY	5
#define RF_DELAY	55

	//sample on rising edge, setup on falling edge
	//CPOL = 0, CPHA=0
	uint8_t i, incoming = 0;

	//Send outgoing byte
	for(i = 0 ; i < 8 ; i++)
	{
		if(data & 0b10000000)
			sbi(TX_PORT, TX_MOSI);
		else
			cbi(TX_PORT, TX_MOSI);

		cbi(TX_PORT, TX_SCK); //TX_SCK = 0;
		_delay_us(RF_DELAY);

		//MISO bit is valid after clock goes going low
		incoming <<= 1;
		if( TX_PORT_PIN & (1<<TX_MISO) ) incoming |= 0x01;

		sbi(TX_PORT, TX_SCK); //TX_SCK = 1;
		_delay_us(RF_DELAY);

		data <<= 1;
	}
	cbi(TX_PORT, TX_SCK); //TX_SCK = 0 after byte sent

	return(incoming);
/* sample on falling edge, setup on rising edge
 * CPOL = 0, CPHA=1
	uint8_t i, incoming = 0;

	//Send outgoing byte
	for(i = 0 ; i < 8 ; i++)
	{
		if(data & 0b10000000)
			sbi(TX_PORT, TX_MOSI);
		else
			cbi(TX_PORT, TX_MOSI);

		sbi(TX_PORT, TX_SCK); //TX_SCK = 1;
		_delay_us(RF_DELAY);

		//MISO bit is valid after clock goes going high
		incoming <<= 1;
		if( TX_PORT_PIN & (1<<TX_MISO) ) incoming |= 0x01;

		cbi(TX_PORT, TX_SCK); //TX_SCK = 0;
		_delay_us(RF_DELAY);

		data <<= 1;
	}

	return(incoming);
 */

#undef TX_PORT
#undef TX_PORT_PIN
#undef TX_PORT_DD
#undef TX_SCK
#undef TX_MISO
#undef TX_MOSI
#undef RF_DELAY
#else
	//use hardware module
#endif
}

//need nordic IQR ISR here, only respond to falling edge of nordic IQR
ISR(INT0_vect)
{
	nordic_IRQ();
}
