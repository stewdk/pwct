/*
 * motor_driver.c
 *
 * Created: 6/4/2012 3:28:23 PM
 *  Author: Stew
 *
 * The motor controller is a Sabertooth 2x60 from Dimension Engineering
 * See the Sabertooth's datasheet for some useful info
 * S1 = PD7/TXD1 (USARTD1)
 * S2 = PD6/RXD1 (USARTD1)
 *
 * DIP switch settings:
 * 1: down
 * 2: down
 * 3: up
 * 4: up
 * 5: up
 * 6: up
 *
 * Address = 128
 */

#include <avr/io.h>
#include "../atmel/usart_driver.h"
#include "motor_driver.h"

USART_data_t USARTD1_data;

void initMotorDriver(void) {
	//Set PD6 and PD7 both as outputs
	PORTD.DIRSET = PIN6_bm | PIN7_bm;
	PORTD.OUTSET = PIN6_bm | PIN7_bm;

	// Use USARTD1 and initialize buffers.
	USART_InterruptDriver_Initialize(&USARTD1_data, &USARTD1, USART_DREINTLVL_HI_gc);

	// USARTD1, 8 Data bits, No Parity, 1 Stop bit.
	USART_Format_Set(USARTD1_data.usart, USART_CHSIZE_8BIT_gc, USART_PMODE_DISABLED_gc, false);

	// Disable RXC interrupt.
	USART_RxdInterruptLevel_Set(USARTD1_data.usart, USART_RXCINTLVL_OFF_gc);

	// Set Baudrate to 9600 bps
	USART_Baudrate_Set(&USARTD1, 3317 , -4);

	// Disable RX, enable TX
	USART_Rx_Disable(USARTD1_data.usart);
	USART_Tx_Enable(USARTD1_data.usart);

	PMIC.CTRL |= PMIC_HILVLEX_bm;

	// Let's hope that global interrupts are already enabled...

	// Set minimum voltage to 18V
	sendMotorCommand(2, 60);

	// Enable serial timeout 500 ms
	sendMotorCommand(14, 5);

	// Enable ramping
	sendMotorCommand(16, 14);
}

void motorEStop(void)
{
	//S2 is an active-low E-stop when the motor driver is in packetized serial mode
	PORTD.OUTCLR = PIN6_bm;
}

void resetMotorEStop(void)
{
	PORTD.OUTSET = PIN6_bm;
}

static void sendMotorPacket(sabertooth_packet *packet)
{
	uint8_t i = 0;
	while (i < sizeof(packet->array))
	{
		if (USART_TXBuffer_PutByte(&USARTD1_data, packet->array[i]))
		{
			i++;
		}
	}
}

void sendMotorCommand(uint8_t command, uint8_t data)
{
	sabertooth_packet packet;
	packet.parts.address = SABERTOOTH_ADDRESS;
	packet.parts.command = command;
	packet.parts.data = data;
	packet.parts.checksum = (packet.parts.address + packet.parts.command + packet.parts.data) & 0x7F;
	sendMotorPacket(&packet);
}

ISR(USARTD1_DRE_vect)
{
	USART_DataRegEmpty(&USARTD1_data);
}
