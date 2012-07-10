/*
 * nordic_driver.h
 *
 *  Created on: Feb 22, 2011
 *      Author: grant
 */

#ifndef NORDIC_DRIVER_H_
#define NORDIC_DRIVER_H_

#include <avr/io.h>

// NORDIC COMMAND WORDS
//	R_REGISTER 0b000XXXXX where XXXXX = 5 bit register map address
#define R_REGISTER_nCmd				0x00
//	W_REGISTER 0b001XXXXX where XXXXX = 5 bit register map address
#define W_REGISTER_nCmd				0x20
#define R_RX_PAYLOAD_nCmd			0x61
#define W_TX_PAYLOAD_nCmd			0xA0
#define FLUSH_TX_nCmd				0xE1
#define FLUSH_RX_nCmd				0xE2
#define REUSE_TX_PL_nCmd			0xE3
#define R_RX_PL_WID_nCmd			0x60
//	W_ACK_PAYLOAD 0b10101PPP where PPP = pipe number to write packet to
#define W_ACK_PAYLOAD_nCmd			0xA8
#define W_TX_PAYLOAD_NOACK_nCmd		0xB0
#define NOP_nCmd					0xFF

// NORDIC REGISTERS (5-bit addresses, 8-bit values)
#define CONFIG_nReg			0x00
#define EN_AA_nReg			0x01
#define EN_RXADDR_nReg		0x02
#define SETUP_AW_nReg		0x03
#define SETUP_RETR_nReg		0x04
#define RF_CH_nReg			0x05
#define RF_SETUP_nReg		0x06
#define STATUS_nReg			0x07
#define OBSERVE_TX_nReg		0x08
#define RPD_nReg			0x09
#define RX_ADDR_P0_nReg		0x0A
#define RX_ADDR_P1_nReg		0x0B
#define RX_ADDR_P2_nReg		0x0C
#define RX_ADDR_P3_nReg		0x0D
#define RX_ADDR_P4_nReg		0x0E
#define RX_ADDR_P5_nReg		0x0F
#define TX_ADDR_nReg		0x10
#define RX_PW_P0_nReg		0x11
#define RX_PW_P1_nReg		0x12
#define RX_PW_P2_nReg		0x13
#define RX_PW_P3_nReg		0x14
#define RX_PW_P4_nReg		0x15
#define RX_PW_P5_nReg		0x16
#define FIFO_STATUS_nReg	0x17
#define DYNPD_nReg			0x1C
#define FEATURE_nReg		0x1D
/*
typedef union {
	uint8_t array[8];
	struct {
		uint8_t LinearActuatorUp;
		uint8_t LinearActuatorDown;
		uint8_t DisableWiredJoystick;
		uint8_t JoystickUp;
		uint8_t JoystickDown;
		uint8_t JoystickLeft;
		uint8_t JoystickRight;
		uint8_t RequestState;
	}parts;
}NORDIC_DATA_PACKET;
*/
typedef union {
	uint8_t array[4];
	struct {
		uint8_t LinearActuatorUp;
		uint8_t LinearActuatorDown;
		uint8_t DisableWiredJoystick;
		uint8_t JoystickUp;

	}parts;
}NORDIC_DATA_PACKET;

typedef struct {
	NORDIC_DATA_PACKET data;
	uint8_t rxpipe;
}NORDIC_PACKET;

int8_t nordic_Initialize(uint8_t receiver);
uint8_t nordic_GetStatus(void);
uint8_t nordic_GetNewPacket(NORDIC_PACKET* packet);
void nordic_GetLastPacket(NORDIC_PACKET* packet);
void nordic_TransmitData(NORDIC_PACKET * packet);
void nordic_PowerDown(void);
void nordic_PowerUp(void);

//nordic_IRQ() is called from an ISR in nordic_hardware_specific.c
//triggered by the falling edge of the IRQ pin from the nordic chip
uint8_t nordic_IRQ(void);

#endif /* NORDIC_DRIVER_H_ */
