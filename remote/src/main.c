/*
 * main.c
 *
 *  Created on: Apr 11, 2011
 *      Author: grant
 */
#include <avr/io.h>
#include "remote_hardware.h"
#include "nordic_driver.h"
#include "util/delay.h"
#include "avr/interrupt.h"
#include "nordic_hardware_specific.h"
#include "string.h"

#define sbi(var, mask)   ((var) |= (uint8_t)(1 << mask))
#define cbi(var, mask)   ((var) &= (uint8_t)~(1 << mask))

int main(void)
{
	NORDIC_PACKET testPacket;
	uint8_t i;

	memset(&testPacket, 0, sizeof(testPacket));

	//enable interrupts
	sei();

	initHardware();
	nordic_Initialize(0);
	sbi(MCUCR, SM1);	//Power Down sleep mode

	nordic_Initialize(0);

/*
	i = 0;
	while(1) {
		tglLED();
		testPacket.data.array[0] = i;
		testPacket.data.array[1] = GetADC5();
		testPacket.data.array[2] = GetADC6();
		nordic_TransmitData(&testPacket);
		_delay_ms(100);
		i++;
	}
*/

	while(1) {
		//go to sleep
		nordic_PowerDown();
		sbi(MCUCR, SE);	//sleep enabled
		asm volatile ("sleep");	//Go to sleep
		cbi(MCUCR, SE);	//sleep disabled
		nordic_PowerUp();

		//set last button pressed time
		resetTimeOut();

		while(1) {
			if(isTimeOut()) {
				//if no buttons are pressed quit out and go to sleep
//				if(!GetButton() && !GetJoyState()) {
				if(!GetJoyState()) {
					break;
				}
				else {
					//send out latest buttons
					//this way if wheelchair doesn't get a packet every timeout period
					//it can assume the remote is out of range
					testPacket.data.array[0] = GetButton() | GetJoyState();
//					testPacket.data.array[0] = GetButton();
//					testPacket.data.array[0] = GetJoyState();
					testPacket.data.array[1] = GetADC5();
					testPacket.data.array[2] = GetADC6();
					nordic_TransmitData(&testPacket);
					resetTimeOut();
				}
			}

			//send only if button was changed
			else if(hasButtonChanged()) {
				clrButtonChanged();
				testPacket.data.array[0] = GetButton() | GetJoyState();
//				testPacket.data.array[0] = GetButton();
//				testPacket.data.array[0] = GetJoyState();
				testPacket.data.array[1] = GetADC5();
				testPacket.data.array[2] = GetADC6();
				nordic_TransmitData(&testPacket);
				resetTimeOut();
			}
		}
	}

	return 0;
}
