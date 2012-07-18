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

void EnableWatchdog(uint8_t timeout)
{
	sbi(WDTCR, timeout);
	sbi(WDTCR, WDE);
}

void DisableWatchdog(void)
{
	asm("wdr");
	/* Clear WDRF in MCUSR */
	MCUSR = 0x00;
	/* Write logical one to WDCE and WDE */
	WDTCR |= (1<<WDCE) | (1<<WDE);
	/* Turn off WDT */
	WDTCR = 0x00;
}

int main(void)
{
	NORDIC_PACKET testPacket;
	//uint8_t i;

	memset(&testPacket, 0, sizeof(testPacket));

	//enable interrupts
	sei();

	initHardware();
	nordic_Initialize(0);
	sbi(MCUCR, SM1);	//Power Down sleep mode

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

	//Start watchdog timer
//	EnableWatchdog(WDP3);	//4second timeout

	while(1) {
		//go to sleep
		nordic_PowerDown();
		clrLED();
//		DisableWatchdog();
		sbi(MCUCR, SE);	//sleep enabled
		asm volatile ("sleep");	//Go to sleep
		cbi(MCUCR, SE);	//sleep disabled
//		EnableWatchdog(WDP3);	//4 second timeout
		nordic_PowerUp();

		//set last button pressed time
		resetTimeOut();

		while(1) {
			tglLED();
//			asm("wdr");	//reset watchdog timer
			if(isTimeOut()) {
				resetTimeOut();
				//if no buttons are pressed quit out and go to sleep
				if(!GetButton() && !GetJoyState()) {
					break;
				}
				else {
					//send out latest buttons
					//this way if wheelchair doesn't get a packet every timeout period
					//it can assume the remote is out of range
					testPacket.data.array[0] = GetButton() | GetJoyState();
					testPacket.data.array[1] = GetADC5();
					testPacket.data.array[2] = GetADC6();
					nordic_TransmitData(&testPacket);
				}
			}

			//send only if button was changed
			else if(hasButtonChanged()) {
				clrButtonChanged();
				testPacket.data.array[0] = GetButton() | GetJoyState();
				testPacket.data.array[1] = GetADC5();
				testPacket.data.array[2] = GetADC6();
				nordic_TransmitData(&testPacket);
				resetTimeOut();
			}
		}
	}

	return 0;
}
