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
#include "menu.h"
#include "../atmel/avr_compiler.h"

#define RXPIPE_INSTRUCTOR_REMOTE 0
#define RXPIPE_STUDENT_JOYSTICK 1

#define SPI_SS_bm   PIN4_bm
#define SPI_MOSI_bm PIN5_bm
#define SPI_MISO_bm PIN6_bm
#define SPI_SCK_bm  PIN7_bm

static volatile NORDIC_PACKET LAST_PACKET;

static uint8_t gStudentForward;
static uint8_t gStudentReverse;
static uint8_t gStudentLeft;
static uint8_t gStudentRight;
static int8_t gStudentSpeed;
static int8_t gStudentDirection;
static uint8_t gInstructorLAUp;
static uint8_t gInstructorLADown;
static uint8_t gInstructorEStop;
static int8_t gInstructorSpeed;
static int8_t gInstructorDirection;
static volatile uint8_t gIsInstructorTimeout;

static void timeoutTimerSetup(TC0_t * tc)
{
	// Packet time-out counter
	tc->CTRLA = TC_CLKSEL_OFF_gc;
	// 1 tick = 32us
	// 7812 ticks = 0.25s
	tc->PER = 7812;
	// Set timer to normal mode
	tc->CTRLB = TC_WGMODE_NORMAL_gc;
	// Set overflow interrupt (level med)
	tc->INTCTRLA = TC_OVFINTLVL_MED_gc;
	// reset packet watchdog timer
	tc->CNT = 0;
	// start timer
	tc->CTRLA = TC_CLKSEL_DIV1024_gc;
}

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

	timeoutTimerSetup(&TCD0);
	timeoutTimerSetup(&TCF0);
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

	configRegValue = 0x0D;  //RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled, two CRC bytes; RX mode
	nordic_WriteRegister(CONFIG_nReg, configRegValue, NULL);

	//enable auto acknowledge on pipe 0 and 1
	nordic_WriteRegister(EN_AA_nReg, 0x03, NULL);

	//enable auto retransmit, try 5 times with delay of 250us
	nordic_WriteRegister(SETUP_RETR_nReg, 0x05, NULL);

	//EN_RXADDR_nReg        Default data pipe 0 and 1 enabled
	//SETUP_AW_nReg         Default address width of 5 bytes

	//Set RF Channel as 0x7C
	nordic_WriteRegister(RF_CH_nReg, 0x7C, NULL);

	//Set output power 0dB, data rate of 250kbps
	nordic_WriteRegister(RF_SETUP_nReg, 0x27, NULL);

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
}

void nordic_Initialize()
{
	hardwareSetup();
	nordicSetup();

	//set default values
	gStudentForward = 0;
	gStudentReverse = 0;
	gStudentLeft = 0;
	gStudentRight = 0;
	gStudentSpeed = 0;
	gStudentDirection = 0;
	gInstructorLAUp = 0;
	gInstructorLADown = 0;
	gInstructorEStop = 0;
	gInstructorSpeed = 0;
	gInstructorDirection = 0;
	gIsInstructorTimeout = 0;
}

uint8_t nordic_getInstructorEStop()
{
	return gInstructorEStop;
}

uint8_t nordic_getInstructorLAUp()
{
	return gInstructorLAUp;
}

uint8_t nordic_getInstructorLADown()
{
	return gInstructorLADown;
}

uint8_t nordic_getStudentForward()
{
	return gStudentForward;
}

uint8_t nordic_getStudentReverse()
{
	return gStudentReverse;
}

uint8_t nordic_getStudentLeft()
{
	return gStudentLeft;
}

uint8_t nordic_getStudentRight()
{
	return gStudentRight;
}

int8_t nordic_getWirelessPropJoySpeed(void)
{
	return gStudentSpeed;
}

int8_t nordic_getWirelessPropJoyDirection(void)
{
	return gStudentDirection;
}

int8_t nordic_getInstructorSpeed()
{
	return gInstructorSpeed;
}

int8_t nordic_getInstructorDirection()
{
	return gInstructorDirection;
}

static void setVariables(uint8_t isTimeout)
{
	if ((LAST_PACKET.data.array[0] & 0b00000001) >> 0 ) {
		gInstructorEStop = 1;
	}
	if (LAST_PACKET.rxpipe == RXPIPE_INSTRUCTOR_REMOTE) {
		gInstructorLAUp = (    (LAST_PACKET.data.array[0] & 0b00000010) >> 1 );
		gInstructorLADown = (  (LAST_PACKET.data.array[0] & 0b00000100) >> 2 );
		gInstructorSpeed = LAST_PACKET.data.array[2];
		gInstructorDirection = LAST_PACKET.data.array[1];
	}

	if (LAST_PACKET.rxpipe == RXPIPE_STUDENT_JOYSTICK) { // Student joystick
		gStudentForward = (  (LAST_PACKET.data.array[0] & 0b00001000) >> 3 );
		gStudentReverse = (  (LAST_PACKET.data.array[0] & 0b00010000) >> 4 );
		gStudentLeft = (     (LAST_PACKET.data.array[0] & 0b00100000) >> 5 );
		gStudentRight = (    (LAST_PACKET.data.array[0] & 0b01000000) >> 6 );
		gStudentSpeed = LAST_PACKET.data.array[2];
		gStudentDirection = LAST_PACKET.data.array[1];
	}
	if (isTimeout) {
		gStudentForward = 0;
		gStudentReverse = 0;
		gStudentLeft = 0;
		gStudentRight = 0;
		gStudentSpeed = 0;
		gStudentDirection = 0;
	}
}

ISR(PORTH_INT0_vect)
{
	uint8_t status;
	uint8_t size = 0;
	uint8_t data[4] = {0,0,0,0};

	//get status
	nordic_SendCommand(NOP_nCmd, NULL, NULL, 0, &status);

	if (status & 0x40) { // Data Ready RX FIFO

		//get payload size
		nordic_SendCommand(R_RX_PL_WID_nCmd, NULL, &size, 1, NULL);
		if (size == sizeof(data)) {
			//get latest packet
			nordic_SendCommand(R_RX_PAYLOAD_nCmd, NULL, data, size, NULL);  //get payload
			LAST_PACKET.data.array[0] = data[0];
			LAST_PACKET.data.array[1] = data[1];
			LAST_PACKET.data.array[2] = data[2];
			LAST_PACKET.data.array[3] = data[3];
			LAST_PACKET.rxpipe = (status & 0x0E) >> 1;

			//reset packet receive time-out
			if (LAST_PACKET.rxpipe == RXPIPE_INSTRUCTOR_REMOTE) {
				TCD0.CNT = 0;
				gIsInstructorTimeout = 0;
			}
			if (LAST_PACKET.rxpipe == RXPIPE_STUDENT_JOYSTICK) {
				TCF0.CNT = 0;
			}
		}
		//clear fifo
		nordic_SendCommand(FLUSH_RX_nCmd, NULL, NULL, 0, NULL);

		//update remote variables
		if (!gIsInstructorTimeout) {
			setVariables(0);
		}
	//} else {
	//	printf("Status=%d\n", status);
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

ISR(TCD0_OVF_vect)	//packet receive time-out
{
	gIsInstructorTimeout = 1;
	LAST_PACKET.data.array[0] = 0;
	LAST_PACKET.data.array[1] = 0;
	LAST_PACKET.data.array[2] = 0;
	LAST_PACKET.data.array[3] = 0;
	LAST_PACKET.rxpipe = RXPIPE_INSTRUCTOR_REMOTE;
	setVariables(1);
	incrementWirelessTimeout();
}

ISR(TCF0_OVF_vect)	//packet receive time-out
{
	LAST_PACKET.data.array[0] = 0;
	LAST_PACKET.data.array[1] = 0;
	LAST_PACKET.data.array[2] = 0;
	LAST_PACKET.data.array[3] = 0;
	LAST_PACKET.rxpipe = RXPIPE_STUDENT_JOYSTICK;
	setVariables(1);
	incrementWirelessTimeout();
}
