/*
 * nordic_driver.c
 *
 *  Created on: Feb 22, 2011
 *      Author: grant
 */
#include <stdio.h>
#include "nordic_driver.h"
#include "nordic_hardware_specific.h"
#include "remote_hardware.h"

static volatile NORDIC_PACKET LAST_PACKET;

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

static inline void setDirTx(void)
{
	standbyMode();
	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; TX mode; PWR_UP bit set
	nordic_WriteRegister(CONFIG_nReg, 0x0A, NULL);
}

static inline void setDirRx(void)
{
	standbyMode();
	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; RX mode; PWR_UP bit set
	nordic_WriteRegister(CONFIG_nReg, 0x0B, NULL);
	activeMode();
}

int8_t nordic_Initialize(uint8_t receiver)
{
	uint8_t configRegValue;
	uint8_t datas[10];
	int8_t err = 0;

	initalizeHardwareForNordic();

	//Initialize Nordic nRF24L01+
	if (receiver) {
		configRegValue = 0x09;	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; RX mode
	} else {
		configRegValue = 0x08;	//RX_DR, TX_DS, MAX_RT on IRQ; CRC enabled; TX mode
	}
	err = nordic_WriteRegister(CONFIG_nReg, configRegValue, NULL);

	//enable auto acknowledge on pipe 0 and 1
	err = nordic_WriteRegister(EN_AA_nReg, 0x03, NULL);

#ifdef INSTRUCTOR_REMOTE
	//enable auto retransmit, try 15 times with delay of 500us
	err = nordic_WriteRegister(SETUP_RETR_nReg, 0x1F, NULL);
#else //STUDENT_JOYSTICK
	//enable auto retransmit, try 1 time with delay of 2500us
	err = nordic_WriteRegister(SETUP_RETR_nReg, 0x91, NULL);
#endif

	//EN_RXADDR_nReg	Default data pipe 0 and 1 enabled
	//    todo: enable only 0 or 1 ?
	//SETUP_AW_nReg		Default address width of 5 bytes

	//Set RF Channel as 0x7C
	err = nordic_WriteRegister(RF_CH_nReg, 0x7C, NULL);

	//Set output power 0dB, data rate of 1Mbps
	err = nordic_WriteRegister(RF_SETUP_nReg, 0x07, NULL);

	//Rx Address data pipe 0
	//ACK comes in on RX data pipe 0
#ifdef INSTRUCTOR_REMOTE
	datas[0] = 0xE7;
	datas[1] = 0xE7;
	datas[2] = 0xE7;
	datas[3] = 0xE7;
	datas[4] = 0xE7;
#else //STUDENT_JOYSTICK
	datas[0] = 0xC2;
	datas[1] = 0xC2;
	datas[2] = 0xC2;
	datas[3] = 0xC2;
	datas[4] = 0xC2;
#endif

	err = nordic_WriteRegisters(RX_ADDR_P0_nReg, datas, 5, NULL);

	//Rx Address data pipe 1
	datas[0] = 0xC2;
	datas[1] = 0xC2;
	datas[2] = 0xC2;
	datas[3] = 0xC2;
	datas[4] = 0xC2;
	err = nordic_WriteRegisters(RX_ADDR_P1_nReg, datas, 5, NULL);

	//Tx Address
#ifdef INSTRUCTOR_REMOTE
	datas[0] = 0xE7;
	datas[1] = 0xE7;
	datas[2] = 0xE7;
	datas[3] = 0xE7;
	datas[4] = 0xE7;
#else //STUDENT_JOYSTICK
	datas[0] = 0xC2;
	datas[1] = 0xC2;
	datas[2] = 0xC2;
	datas[3] = 0xC2;
	datas[4] = 0xC2;
#endif
	err = nordic_WriteRegisters(TX_ADDR_nReg, datas, 5, NULL);

	//Set Payload width of 4 bytes
	err = nordic_WriteRegister(RX_PW_P0_nReg, sizeof(LAST_PACKET.data.array), NULL);
	err = nordic_WriteRegister(RX_PW_P1_nReg, sizeof(LAST_PACKET.data.array), NULL);

	//clear fifos (necessary for wdt/soft reset)
	nordic_SendCommand(FLUSH_RX_nCmd, NULL, NULL, 0, NULL);
	nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);

	//clear interrupts (necessary for wdt/soft reset)
	nordic_WriteRegister(STATUS_nReg, 0x70, NULL);

	configRegValue |= 0x02; //PWR_UP bit set
	err = nordic_WriteRegister(CONFIG_nReg, configRegValue, NULL);

	//wait for startup
	_delay_us(1500);

	if (receiver) {
		activeMode();	//start receiving
	}

	return err;
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
inline uint8_t nordic_IRQ(void)
{
	uint8_t status, previousMode, size = 0;
	uint8_t data[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};

	//get status
	previousMode = standbyMode();
	nordic_SendCommand(NOP_nCmd, NULL, NULL, 0, &status);

	if (status & 0x40) { // Data Ready RX FIFO
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
	}
	if (status & 0x20) { // Data Sent TX FIFO
		nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);
	}
	if (status & 0x10) { // Maximum number of TX retransmits
		nordic_SendCommand(FLUSH_TX_nCmd, NULL, NULL, 0, NULL);
		setLED();
	} else {
		clrLED();
	}

	//clear interrupts
	nordic_WriteRegister(STATUS_nReg, status & 0x70, NULL);
	setMode(previousMode);

	return status;
}
