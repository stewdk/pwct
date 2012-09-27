/*
 * nordic_driver.c
 *
 *  Created on: Feb 22, 2011
 *      Author: grant
 */

#include <stdio.h>
#include <string.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include "nordic_driver.h"
#include "PWCT_io.h"

#define SPI_SS_bm   PIN4_bm
#define SPI_MOSI_bm PIN5_bm
#define SPI_MISO_bm PIN6_bm
#define SPI_SCK_bm  PIN7_bm

static volatile NORDIC_PACKET LAST_PACKET;

static uint8_t INSTRUCTOR_FORWARD;
static uint8_t INSTRUCTOR_REVERSE;
static uint8_t INSTRUCTOR_LEFT;
static uint8_t INSTRUCTOR_RIGHT;
static uint8_t INSTRUCTOR_LA_UP;
static uint8_t INSTRUCTOR_LA_DOWN;
static uint8_t INSTRUCTOR_ESTOP;

static inline void hardwareSetup()
{
	// SPI prescaler = div128, enable, master
	SPIF.CTRL = SPI_PRESCALER_DIV128_gc | SPI_ENABLE_bm | SPI_MASTER_bm | SPI_MODE_0_gc;

	// No SPI interrupt
	SPIF.INTCTRL = SPI_INTLVL_OFF_gc;

	// MOSI and SCK as output
	PORTF.DIRSET = SPI_MOSI_bm | SPI_SCK_bm;

	// Set chip enable (CE) pin as output (not a part of SPI)
	PORTE.OUTCLR = PIN7_bm;
	PORTE.DIRSET = PIN7_bm;

	// Setup IRQ pin interrupt
	PORTH.DIRCLR = PIN2_bm;
	PORTH.PIN2CTRL = PORT_ISC_FALLING_gc;
	PORTH.INTCTRL = PORT_INT0LVL_MED_gc;
	PORTH.INT0MASK = PIN2_bm;

	// Enable med level interrupt
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;

	// Packet time-out counter
	TCF0.CTRLA = TC_CLKSEL_OFF_gc;
	// 1 tick = 32us
	// 3125 ticks = 0.1s
	TCF0.PER = 3125;
	// Set timer to normal mode
	TCF0.CTRLB = TC_WGMODE_NORMAL_gc;
	// Set overflow interrupt (level med)
	TCF0.INTCTRLA = TC_OVFINTLVL_MED_gc;
	// reset packet watchdog timer
	TCF0.CNT = 0;
	// start timer
	TCF0.CTRLA = TC_CLKSEL_DIV1024_gc;
}

// CS line must be pulled low before calling this function and released when finished
static uint8_t SPI_TransceiveByte(uint8_t TXdata)
{
	// Send pattern
	SPIF.DATA = TXdata;

	// Wait for transmission complete
	while (!(SPIF.STATUS & SPI_IF_bm)) {

	}

	// Read received data
	uint8_t result = SPIF.DATA;

	return (result);
}

// pull CS low
static inline void chipSelect(void)
{
	PORTF.OUTCLR = PIN4_bm;
	PORTF.DIRSET = PIN4_bm;
}

// release CS
static inline void chipRelease(void)
{
	PORTF.OUTSET = PIN4_bm;
	PORTF.DIRCLR = PIN4_bm;
}

//set CE high
static inline void activeMode(void)
{
	PORTE.OUTSET = PIN7_bm;
}

static int8_t nordic_SendCommand(uint8_t cmd, uint8_t *txdata, uint8_t *rxdata, uint8_t dataSize, uint8_t *status)
{
	uint8_t i;
	uint8_t rx;
	uint8_t data;
	uint8_t statusFake;
	int8_t err = 0;

	//check to make sure parameters are valid
	if (status == NULL) {
		status = &statusFake;
	}

	chipSelect();

	//send command
	*status = SPI_TransceiveByte(cmd);

	//send/receive LSByte first
	if (dataSize != 0) {
		i = dataSize;
		do {
			i--;
			if (txdata == NULL) {
				data = 0;
			} else {
				data = txdata[i];
			}
			rx = SPI_TransceiveByte(data);
			if (rxdata != NULL) {
				rxdata[i] = rx;
			}
		} while(i != 0);
	}

	chipRelease();

	return err;
}

//Write a register that contains a single byte of data
static inline int8_t nordic_WriteRegister(uint8_t reg, uint8_t data, uint8_t *status)
{
	return nordic_SendCommand(W_REGISTER_nCmd | reg, &data, NULL, 1, status);
}

//Writes a register with N bytes of data
static inline int8_t nordic_WriteRegisters(uint8_t reg, uint8_t *data, uint8_t size, uint8_t *status)
{
	return nordic_SendCommand(W_REGISTER_nCmd | reg, data, NULL, size, status);
}

static void nordicSetup()
{
	uint8_t datas[10];
	uint8_t configRegValue;

	configRegValue = 0x09;  //RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; RX mode
	nordic_WriteRegister(CONFIG_nReg, configRegValue, NULL);

	//enable auto acknowledge on pipe 0 and 1
	nordic_WriteRegister(EN_AA_nReg, 0x03, NULL);

	//enable auto retransmit, try 5 times with delay of 250us
	nordic_WriteRegister(SETUP_RETR_nReg, 0x05, NULL);

	//EN_RXADDR_nReg        Default data pipe 0 and 1 enabled
	//SETUP_AW_nReg         Default address width of 5 bytes

	//Set RF Channel as 0x02
	nordic_WriteRegister(RF_CH_nReg, 0x02, NULL);

	//Set output power 0dB, data rate of 1Mbps
	nordic_WriteRegister(RF_SETUP_nReg, 0x07, NULL);

	//Rx Address data pipe 0
	datas[0] = 0xE7;
	datas[1] = 0xE7;
	datas[2] = 0xE7;
	datas[3] = 0xE7;
	datas[4] = 0xE7;
	nordic_WriteRegisters(RX_ADDR_P0_nReg, datas, 5, NULL);

	//Rx Address data pipe 1
	datas[0] = 0xC2;
	datas[1] = 0xC2;
	datas[2] = 0xC2;
	datas[3] = 0xC2;
	datas[4] = 0xC2;
	nordic_WriteRegisters(RX_ADDR_P1_nReg, datas, 5, NULL);

	//Set Payload width of 4 bytes
	nordic_WriteRegister(RX_PW_P0_nReg, sizeof(LAST_PACKET.data.array), NULL);
	nordic_WriteRegister(RX_PW_P1_nReg, sizeof(LAST_PACKET.data.array), NULL);

	//clear fifos (necessary for wdt/soft reset)
	nordic_SendCommand(FLUSH_RX_nCmd, NULL, NULL, 0, NULL);
	nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);

	//clear interrupts (necessary for wdt/soft reset)
	nordic_WriteRegister(STATUS_nReg, 0x70, NULL);

	configRegValue |= 0x02; //PWR_UP bit set
	nordic_WriteRegister(CONFIG_nReg, configRegValue, NULL);

	//wait for startup
	_delay_us(1500);

	activeMode(); //start receiving

	//set default values
	INSTRUCTOR_FORWARD = 0;
	INSTRUCTOR_REVERSE = 0;
	INSTRUCTOR_LEFT = 0;
	INSTRUCTOR_RIGHT = 0;
	INSTRUCTOR_LA_UP = 0;
	INSTRUCTOR_LA_DOWN = 0;
	INSTRUCTOR_ESTOP = 0;
}

void nordic_Initialize()
{
	hardwareSetup();
	nordicSetup();
}

uint8_t nordic_getInstructorEStop()
{
	return INSTRUCTOR_ESTOP;
}

uint8_t nordic_getInstructorLAUp()
{
	return INSTRUCTOR_LA_UP;
}

uint8_t nordic_getInstructorLADown()
{
	return INSTRUCTOR_LA_DOWN;
}

uint8_t nordic_getInstructorForward()
{
	return INSTRUCTOR_FORWARD;
}

uint8_t nordic_getInstructorReverse()
{
	return INSTRUCTOR_REVERSE;
}

uint8_t nordic_getInstructorLeft()
{
	return INSTRUCTOR_LEFT;
}

uint8_t nordic_getInstructorRight()
{
	return INSTRUCTOR_RIGHT;
}

uint8_t nordic_getWirelessPropJoySpeed(void)
{
	return LAST_PACKET.data.array[2];
}

uint8_t nordic_getWirelessPropJoyDirection(void)
{
	return LAST_PACKET.data.array[1];
}

static void SetInstructorRemote(void)
{
	INSTRUCTOR_ESTOP = (    (LAST_PACKET.data.array[0] & 0b00000001) >> 0 );
	INSTRUCTOR_LA_UP = (    (LAST_PACKET.data.array[0] & 0b00000010) >> 1 );
	INSTRUCTOR_LA_DOWN = (  (LAST_PACKET.data.array[0] & 0b00000100) >> 2 );
	INSTRUCTOR_FORWARD = (  (LAST_PACKET.data.array[0] & 0b00001000) >> 3 );
	INSTRUCTOR_REVERSE = (  (LAST_PACKET.data.array[0] & 0b00010000) >> 4 );
	INSTRUCTOR_LEFT = (     (LAST_PACKET.data.array[0] & 0b00100000) >> 5 );
	INSTRUCTOR_RIGHT = (    (LAST_PACKET.data.array[0] & 0b01000000) >> 6 );
}

ISR(PORTH_INT0_vect)
{
	uint8_t status;
	uint8_t size = 0;
	uint8_t data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	//get status
	nordic_SendCommand(NOP_nCmd, NULL, NULL, 0, &status);

	if (status & 0x40) { // Data Ready RX FIFO

		//reset packet receive time-out
		TCF0.CNT = 0;

		//get latest packet
		nordic_SendCommand(R_RX_PL_WID_nCmd, NULL, &size, 1, NULL);     //get payload size
		if (size > sizeof(data)) {
			size = sizeof(data);
		}
		if (size != 0) {
			nordic_SendCommand(R_RX_PAYLOAD_nCmd, NULL, data, size, NULL);  //get payload
			LAST_PACKET.data.array[0] = data[0];
			LAST_PACKET.data.array[1] = data[1];
			LAST_PACKET.data.array[2] = data[2];
			LAST_PACKET.data.array[3] = data[3];
			LAST_PACKET.rxpipe = (status & 0x0E) >> 1;
		}
		//clear fifo
		nordic_SendCommand(FLUSH_RX_nCmd, NULL, NULL, 0, NULL);

		//update remote variables
		SetInstructorRemote();
	}
	if (status & 0x20) { // Data Sent TX FIFO
		nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);
	}
	if (status & 0x10) { // Maximum number of TX retransmits
		nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);
	}

	//clear interrupts
	nordic_WriteRegister(STATUS_nReg, status & 0x70, NULL);
}

ISR(TCF0_OVF_vect)	//packet receive time-out
{
	LAST_PACKET.data.array[0] = 0;
	LAST_PACKET.data.array[1] = 0;
	LAST_PACKET.data.array[2] = 0;
	LAST_PACKET.data.array[3] = 0;
	LAST_PACKET.rxpipe = 0;
	SetInstructorRemote();
}
