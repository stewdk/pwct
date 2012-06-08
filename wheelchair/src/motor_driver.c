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

void sendMotorPacket(sabertooth_packet *packet)
{
	packet->parts.checksum = (packet->parts.address + packet->parts.command + packet->parts.data) & 0x7F;
	uint8_t i = 0;
	while (i < sizeof(packet->array))
	{
		if (USART_TXBuffer_PutByte(&USARTD1_data, packet->array[i]))
		{
			i++;
		}
	}
}

ISR(USARTD1_DRE_vect)
{
	USART_DataRegEmpty(&USARTD1_data);
}
