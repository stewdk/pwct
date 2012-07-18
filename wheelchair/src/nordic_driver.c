/*
 * nordic_driver.c
 *
 *  Created on: Feb 22, 2011
 *      Author: grant
 */
#include <stdio.h>
#include <string.h>
#include "nordic_driver.h"
#include "nordic_hardware_specific.h"
#include "PWCT_io.h"

#define USE_ENHANCED_SHOCKBURST	1

static volatile int8_t RX_DATA_READY_FLAG = 0;
static volatile int8_t TX_DATA_SENT_FLAG = 0;
static volatile int8_t MAX_RETRANSMITS_READY_FLAG = 0;
static volatile NORDIC_PACKET LAST_PACKET;

static uint8_t INSTRUCTOR_FORWARD;
static uint8_t INSTRUCTOR_REVERSE;
static uint8_t INSTRUCTOR_LEFT;
static uint8_t INSTRUCTOR_RIGHT;
static uint8_t INSTRUCTOR_LA_UP;
static uint8_t INSTRUCTOR_LA_DOWN;
static uint8_t INSTRUCTOR_ESTOP;

static void nordic_CopyPacket(NORDIC_PACKET* dest, volatile NORDIC_PACKET* src)
{
	AVR_ENTER_CRITICAL_REGION();
	dest->data.array[0] = src->data.array[0];
	dest->data.array[1] = src->data.array[1];
	dest->data.array[2] = src->data.array[2];
	dest->data.array[3] = src->data.array[3];
	dest->rxpipe = src->rxpipe;
	AVR_LEAVE_CRITICAL_REGION();
}

static void nordic_ClearPacket(volatile NORDIC_PACKET* packet)
{
	AVR_ENTER_CRITICAL_REGION();
	packet->data.array[0] = 0;
	packet->data.array[1] = 0;
	packet->data.array[2] = 0;
	packet->data.array[3] = 0;
	packet->rxpipe = 0;
	AVR_LEAVE_CRITICAL_REGION();
}

//make sure txdata and rxdata are at least of length dataSize
static int8_t nordic_SendCommand(uint8_t cmd, uint8_t *txdata, uint8_t *rxdata, uint8_t dataSize, uint8_t *status)
{
	uint8_t i;
	uint8_t rx;
	uint8_t data;
	uint8_t statusFake;
	int8_t err = 0;

	//check to make sure parameters are valid
	if(status == NULL) {
		status = &statusFake;
	}

	standbyMode();

	//	_delay_us(4);
	chipSelect();

	//send command
	*status = SPI_TransceiveByte(cmd);

	//send/receive LSByte first
	if(dataSize != 0) {
		i = dataSize;
		do {
			i--;
			if(txdata == NULL) {
				data = 0;
			}
			else {
				data = txdata[i];
			}
			rx = SPI_TransceiveByte(data);
			if(rxdata != NULL) {
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

//Read a register that contains a single byte of data
static inline int8_t nordic_ReadRegister(uint8_t reg, uint8_t *data, uint8_t *status)
{
	return nordic_SendCommand(R_REGISTER_nCmd | reg, NULL, data, 1, status);
}

//Read a register with N bytes of data
static inline int8_t nordic_ReadRegisters(uint8_t reg, uint8_t *data, uint8_t size, uint8_t *status)
{
	return nordic_SendCommand(R_REGISTER_nCmd | reg, NULL, data, size, status);
}

static void setDirTx(void)
{
	standbyMode();
	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; TX mode; PWR_UP bit set
	nordic_WriteRegister(CONFIG_nReg, 0x0A, NULL);
}

/*
static void setDirRx(void)
{
	standbyMode();
	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; RX mode; PWR_UP bit set
	nordic_WriteRegister(CONFIG_nReg, 0x0B, NULL);
	activeMode();
}
*/

int8_t nordic_Initialize(uint8_t receiver)
{
	//todo: EN_RXADDR
	//After wednesday: data pipe 0 and 1 with unique 5-byte address
	//Before wednesday: get prop. data from wireless
	uint8_t data;
	uint8_t datas[10];
	int8_t err = 0;

	initalizeHardwareForNordic();

	//Initialize Nordic nRF24L01+
	if (receiver) {
		data = 0x09;	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; RX mode
	} else {
		data = 0x08;	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; TX mode
	}
	err = nordic_WriteRegister(CONFIG_nReg, data, NULL);

#if USE_ENHANCED_SHOCKBURST
	//enable auto acknowledge on pipe 0
	err = nordic_WriteRegister(EN_AA_nReg, 0x01, NULL);

	//enable auto retransmit, try 5 times with delay of 250us
	err = nordic_WriteRegister(SETUP_RETR_nReg, 0x05, NULL);
#else
	//Disable auto acknowledge
	err = nordic_WriteRegister(EN_AA_nReg, 0x00, NULL);

	//Disable auto retransmit
	err = nordic_WriteRegister(SETUP_RETR_nReg, 0x00, NULL);
#endif
	//EN_RXADDR_nReg	Default data pipe 0 and 1 enabled
	//SETUP_AW_nReg		Default address width of 5 bytes
	//SETUP_RETR_nReg	Only used with auto acknowledge

	//Set RF Channel as 0x02
	err = nordic_WriteRegister(RF_CH_nReg, 0x02, NULL);


	err = nordic_WriteRegister(RF_SETUP_nReg, 0x07, NULL);	//Set output power 0dB, data rate of 1MHz
//	err = nordic_WriteRegister(RF_SETUP_nReg, 0x01, NULL);	//Set output power -18dB, data rate of 1MHz


	//Set Payload of 4 bytes
	err = nordic_WriteRegister(RX_PW_P0_nReg, sizeof(LAST_PACKET.data.array), NULL);

	//Rx Address data pipe
	datas[0] = datas[1] = datas[2] = datas[3] = 0xE7;
	err = nordic_WriteRegisters(RX_ADDR_P0_nReg, datas, 4, NULL);

	//Tx Address data pipe
	datas[0] = datas[1] = datas[2] = datas[3] = 0xE7;
	err = nordic_WriteRegisters(TX_ADDR_nReg, datas, 4, NULL);

	//clear fifos (necessary for wdt/soft reset)
	nordic_SendCommand(FLUSH_RX_nCmd, NULL, NULL, 0, NULL);
	nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);

	//clear interrupts (necessary for wdt/soft reset)
	nordic_WriteRegister(STATUS_nReg, 0x70, NULL);

	//RX on IRQ, PWR_UP bit set, CRC enabled, RX mode
	if (receiver) {
		data = 0x0B;	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; RX mode; PWR_UP bit set
	} else {
		data = 0x0A;	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; TX mode; PWR_UP bit set
	}
	err = nordic_WriteRegister(CONFIG_nReg, data, NULL);

	//wait for startup
	_delay_us(1500);

	if (receiver) {
		activeMode();	//start receiving
	}

	//set default values
	INSTRUCTOR_FORWARD = 0;
	INSTRUCTOR_REVERSE = 0;
	INSTRUCTOR_LEFT = 0;
	INSTRUCTOR_RIGHT = 0;
	INSTRUCTOR_LA_UP = 0;
	INSTRUCTOR_LA_DOWN = 0;
	INSTRUCTOR_ESTOP = 0;

	return err;
}

uint8_t getInstructorEStop()
{
	return INSTRUCTOR_ESTOP;
}
uint8_t getInstructorLAUp()
{
	return INSTRUCTOR_LA_UP;
}
uint8_t getInstructorLADown()
{
	return INSTRUCTOR_LA_DOWN;
}
uint8_t getInstructorForward()
{
	return INSTRUCTOR_FORWARD;
}
uint8_t getInstructorReverse()
{
	return INSTRUCTOR_REVERSE;
}
uint8_t getInstructorLeft()
{
	return INSTRUCTOR_LEFT;
}
uint8_t getInstructorRight()
{
	return INSTRUCTOR_RIGHT;
}

uint8_t getWirelessPropJoySpeed(void)
{
	uint8_t speed;
	AVR_ENTER_CRITICAL_REGION();
	speed = LAST_PACKET.data.array[2];
	AVR_LEAVE_CRITICAL_REGION();
	return speed;
}

uint8_t getWirelessPropJoyDirection(void)
{
	uint8_t dir;
	AVR_ENTER_CRITICAL_REGION();
	dir = LAST_PACKET.data.array[1];
	AVR_LEAVE_CRITICAL_REGION();
	return dir;
}

uint8_t nordic_GetStatus(void)
{
	uint8_t status;
	nordic_SendCommand(NOP_nCmd, NULL, NULL, 0, &status);
	return status;
}

//stores received data in *data, size is the most data it will return
//the return value is the number of data bytes stored in data
uint8_t nordic_GetNewPacket(NORDIC_PACKET* packet)
{
	uint8_t packetFound = 0;

	if(RX_DATA_READY_FLAG) {
		AVR_ENTER_CRITICAL_REGION();	//clear flag and copy packet is a critical section
		RX_DATA_READY_FLAG = 0;
		nordic_CopyPacket(packet, &LAST_PACKET);
		AVR_LEAVE_CRITICAL_REGION();
		packetFound = 1;
	}
	return packetFound;
}

//stores received data in *data, size is the most data it will return
//the return value is the number of data bytes stored in data
void nordic_GetLastPacket(NORDIC_PACKET* packet)
{
	nordic_CopyPacket(packet, &LAST_PACKET);
	if(RX_DATA_READY_FLAG) {
		AVR_ENTER_CRITICAL_REGION();	//clear flag is a critical section
		RX_DATA_READY_FLAG = 0;
		AVR_LEAVE_CRITICAL_REGION();
	}
}

//clears the last rx packet, called if packet watchdog times out
void ClearLastPacket(void)
{
	nordic_ClearPacket(&LAST_PACKET);
}

void SetInstructorRemote(void)
{
	NORDIC_PACKET packet;
	uint8_t newEStopState;

	//parse the packet, call appropriate setter functions
	nordic_CopyPacket(&packet, &LAST_PACKET);

	//make sure to only send one e-stop pulse
	newEStopState = (packet.data.array[0] & 0b00000001) >> 0;
	if ((newEStopState == 0) && (INSTRUCTOR_ESTOP != newEStopState)) {
		PulsePGDTEstop();
	}
	INSTRUCTOR_ESTOP = newEStopState;
	INSTRUCTOR_LA_UP = (	(packet.data.array[0] & 0b00000010) >> 1 );
	INSTRUCTOR_LA_DOWN = (	(packet.data.array[0] & 0b00000100) >> 2 );
	INSTRUCTOR_FORWARD = (	(packet.data.array[0] & 0b00001000) >> 3 );
	INSTRUCTOR_REVERSE = (	(packet.data.array[0] & 0b00010000) >> 4 );
	INSTRUCTOR_LEFT = (		(packet.data.array[0] & 0b00100000) >> 5 );
	INSTRUCTOR_RIGHT = (	(packet.data.array[0] & 0b01000000) >> 6 );
}

//This sends out the data in txdata, leaves chip in standby tx mode
void nordic_TransmitData(NORDIC_PACKET * packet)
{
	nordic_WriteRegister(STATUS_nReg, 0x70, NULL);	//Clear any interrupts

	setDirTx();	//set to Tx mode, powered up

	nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);	//Clear TX Fifo

	// put dataSize bytes from txdata into the tx fifo
	nordic_SendCommand(W_TX_PAYLOAD_nCmd, packet->data.array, NULL, sizeof(packet->data.array), NULL);

	//Pulse CE to start transmission for at least 10us
    activeMode();
    _delay_us(50);
    standbyMode();
}

//powers down nordic chip
void nordic_PowerDown(void)
{
	uint8_t configReg;
	standbyMode();
	nordic_ReadRegister(CONFIG_nReg, &configReg, NULL);
	nordic_WriteRegister(CONFIG_nReg, configReg & 0b11111101, NULL);
}

//powers nordic up, sets active if nordic is configured as a receiver
void nordic_PowerUp(void)
{
	uint8_t configReg;
	standbyMode();
	nordic_ReadRegister(CONFIG_nReg, &configReg, NULL);
	nordic_WriteRegister(CONFIG_nReg, configReg | 0x2, NULL);
	//wait for startup
	_delay_us(1500);

	if (configReg & 0x01) {
		activeMode();	//start receiving
	}
}

//Nordic IRQ pin interrupt
uint8_t nordic_IRQ(void)
{
	uint8_t status, previousMode, size = 0;
	uint8_t data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	//get status
	previousMode = standbyMode();
	nordic_SendCommand(NOP_nCmd, NULL, NULL, 0, &status);

	if(status & 0x40) { // Data Ready RX FIFO
		RX_DATA_READY_FLAG = 1;
		//get latest packet
		nordic_SendCommand(R_RX_PL_WID_nCmd, NULL, &size, 1, NULL);	//get payload size
		if (size > sizeof(data)) {
			size = sizeof(data);
		}
		if (size != 0) {
			nordic_SendCommand(R_RX_PAYLOAD_nCmd, NULL, data, size, NULL);	//get payload
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
	if(status & 0x20) { // Data Sent TX FIFO
		TX_DATA_SENT_FLAG = 1;
		nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);
	}
	if(status & 0x10) { // Maximum number of TX retransmits
		MAX_RETRANSMITS_READY_FLAG = 1;
		nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);
	}

	//clear interrupts
	nordic_WriteRegister(STATUS_nReg, status & 0x70, NULL);
	setMode(previousMode);

	return status;
}
