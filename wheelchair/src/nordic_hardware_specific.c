/*
 * nordic_hardware_specific.c
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */
#include "nordic_hardware_specific.h"
#include "util.h"
#include "TC_driver.h"

#define NORDIC_INTERNAL		1
#define NORDIC_MODULE		2

#define NORDIC_OPTION NORDIC_INTERNAL
//#define NORDIC_OPTION NORDIC_MODULE

//watchdog timer 1tick = 32us
//46900 ticks = 1.5s
#define WATCHDOG_PERIOD	46900

#include "util.h"
#include "spi_driver.h"

SPI_Master_t nordic_spi;

//pull CS low
inline void chipSelect(void)
{
#if NORDIC_OPTION == NORDIC_INTERNAL
	PORTF.OUTCLR = PIN4_bm;
	PORTF.DIRSET = PIN4_bm;
#else	//NORDIC_MODULE
	PORTE.OUTCLR = PIN6_bm;
	PORTE.DIRSET = PIN6_bm;
#endif

}

//release SS
inline void chipRelease(void)
{
#if NORDIC_OPTION == NORDIC_INTERNAL
	PORTF.OUTSET = PIN4_bm;
	PORTF.DIRCLR = PIN4_bm;
#else	//NORDIC_MODULE
	PORTF.OUTSET = PIN6_bm;
	PORTE.DIRCLR = PIN6_bm;
#endif
}

//set CE low
inline uint8_t standbyMode(void)
{
	uint8_t mode;
	mode = PORTE.OUT & PIN7_bm;
	PORTE.OUTCLR = PIN7_bm;
	return mode;
}

//set CE high
inline void activeMode(void)
{
	PORTE.OUTSET = PIN7_bm;
}

//set CE low if mode == 0, else set CE high
inline void setMode(uint8_t mode)
{
	PORTE.OUTSET = mode;
}

//Initialize microcontroller pin directions, spi, interrupts
void initalizeHardwareForNordic(void)
{
	SPI_MasterInit(&nordic_spi, &SPIF, &PORTF, false, SPI_MODE_0_gc,
					SPI_INTLVL_OFF_gc, false, SPI_PRESCALER_DIV128_gc);

	//set chip enable (CE) pin as output
	PORTE.OUTCLR = PIN7_bm;
	PORTE.DIRSET = PIN7_bm;

	//Setup IRQ pin interrupt
	PORTH.DIRCLR = PIN2_bm;
	PORTH.PIN2CTRL = PORT_ISC_FALLING_gc;
	PORTH.INTCTRL = PORT_INT0LVL_MED_gc;
	PORTH.INT0MASK = PIN2_bm;

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_MEDLVLEX_bp;

	//turn off timer
	TC0_ConfigClockSource( &TCF0, TC_CLKSEL_OFF_gc );
	TC_SetPeriod( &TCF0, WATCHDOG_PERIOD );
	//Set timer in normal mode
	TC0_ConfigWGM( &TCF0, TC_WGMODE_NORMAL_gc );
	//set overflow interrupt level
	TC0_SetOverflowIntLevel(&TCF0, TC_OVFINTLVL_LO_gc);
	//reset packet watchdog timer
	TCF0.CNT = 0;
	//start timer
	TC0_ConfigClockSource( &TCF0, TC_CLKSEL_DIV1024_gc );
}

uint8_t SPI_TransceiveByte(uint8_t data)
{
	return SPI_MasterTransceiveByte(&nordic_spi, data);
}

ISR(PORTH_INT0_vect)
{
	uint8_t flags;
//	dbgLEDtgl();
	flags = nordic_IRQ();

	//check flags variable for RX flag
	if (flags & 0x40) {
		//reset packet watchdog timer
		TCF0.CNT = 0;
		//update remote variables
		SetInstructorRemote();
	}
}

ISR(TCF0_OVF_vect)	//packet watchdog overflow
{
	ClearLastPacket();
	SetInstructorRemote();
}
