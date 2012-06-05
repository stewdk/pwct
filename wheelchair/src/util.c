/*
 * util.c
 *
 *  Created on: Nov 10, 2010
 *      Author: grant
 */
#include <stdint.h>
#include <avr/io.h>
#include "../atmel/usart_driver.h"
#include <avr/interrupt.h>
#include <stdbool.h>
#include "string.h"
#include <stdio.h>
#include "util.h"

static int uart_putchar (char c, FILE *stream);

/*! USART data struct. */
USART_data_t USART_data;
static FILE mystdout = FDEV_SETUP_STREAM (uart_putchar, NULL, _FDEV_SETUP_WRITE);

void dbgLEDinit(void)
{
	PORTD.OUTCLR = PIN0_bm;
	PORTD.DIRSET = PIN0_bm;
}

void inline dbgLEDset(void)
{
	PORTD.OUTSET = PIN0_bm;
}

void inline dbgLEDclr(void)
{
	PORTD.OUTCLR = PIN0_bm;
}

void inline dbgLEDtgl(void)
{
	PORTD.OUTTGL = PIN0_bm;
}


void dbgUSARTinit(void)
{
	stdout = &mystdout;

	/* This PORT setting is only valid to USARTC0 if other USARTs is used a
	 * different PORT and/or pins are used. */
  	/* PD3 (TXD0) as output. */
	PORTD.DIRSET   = PIN3_bm;
	/* PD2 (RXD0) as input. */
	PORTD.DIRCLR   = PIN2_bm;

	/* Use USARTC0 and initialize buffers. */
	USART_InterruptDriver_Initialize(&USART_data, &USARTD0, USART_DREINTLVL_LO_gc);

	/* USARTC0, 8 Data bits, No Parity, 1 Stop bit. */
	USART_Format_Set(USART_data.usart, USART_CHSIZE_8BIT_gc,
                     USART_PMODE_DISABLED_gc, false);

	/* Enable RXC interrupt. */
	USART_RxdInterruptLevel_Set(USART_data.usart, USART_RXCINTLVL_LO_gc);

	/* Set Baudrate to 115200 bps */
	//USART_Baudrate_Set(USART_data.usart, 1047 , -6);

	/* Set Baudrate to 38400 bps */
	USART_Baudrate_Set(USART_data.usart, 3269 , -6);

	/* Enable both RX and TX. */
	USART_Rx_Enable(USART_data.usart);
	USART_Tx_Enable(USART_data.usart);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_LOLVLEX_bm;

}

void dbgPutChar(char c)
{
	bool byteBuffered = false;
	while (byteBuffered == false) {
		byteBuffered = USART_TXBuffer_PutByte(&USART_data, c);
	}
}

void dbgPutStr(char *str)
{
	uint8_t i = 0;
	uint8_t len;
	bool byteToBuffer;
	len = strlen(str);

	while (i < len) {
		byteToBuffer = USART_TXBuffer_PutByte(&USART_data, str[i]);
		if(byteToBuffer){
			i++;
		}
	}
}

//0 for success, -1 for no available bytes
int8_t dbgGetCharNonblocking(char *c)
{
	int8_t err = 0;

	if (USART_RXBufferData_Available(&USART_data)) {
		*c = USART_RXBuffer_GetByte(&USART_data);
	}
	else {
		err = -1;	//no bytes available
	}
	return err;
}

//0 for success, -1 for no available bytes
//max length is the max number of characters in string, so actually string length
//would be maxLength+1
//return 0 success, -1  didn't reach end of string
int8_t dbgGetStrNonblocking(char *str, uint8_t maxLength)
{
	uint8_t i = 0, err = 0;

	do {
		if (USART_RXBufferData_Available(&USART_data)) {
			str[i] = USART_RXBuffer_GetByte(&USART_data);
			i++;
		}
		else {
			str[i] = 0;
			err = -1;
			break;
		}
	} while (str[i-1] != 0);

	return err;
}

/*! \brief Receive complete interrupt service routine.
 *
 *  Receive complete interrupt service routine.
 *  Calls the common receive complete handler with pointer to the correct USART
 *  as argument.
 */
ISR(USARTF0_RXC_vect)
{
	USART_RXComplete(&USART_data);

	//echo
	if (USART_RXBufferData_Available(&USART_data)) {
		dbgPutChar(USART_RXBuffer_GetByte(&USART_data));
	}
}


/*! \brief Data register empty  interrupt service routine.
 *
 *  Data register empty  interrupt service routine.
 *  Calls the common data register empty complete handler with pointer to the
 *  correct USART as argument.
 */
ISR(USARTD0_DRE_vect)
{
	USART_DataRegEmpty(&USART_data);
}

//-----------------------------------------------------------------------
//put a byte in the passed file stream
static int uart_putchar (char c, FILE *stream)
{
	//replace new line with carriage return
    if (c == '\n')
        uart_putchar('\r', stream);

    // Wait for the transmit buffer to be empty
    while ( !( USART_data.usart->STATUS & USART_DREIF_bm) );

    // Put byte into the transmit buffer
    USART_data.usart->DATA = c;

    return 0;
}
