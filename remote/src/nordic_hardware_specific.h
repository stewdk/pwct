/*
 * nordic_hardware_specific.h
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */

#ifndef NORDIC_HARDWARE_SPECIFIC_H_
#define NORDIC_HARDWARE_SPECIFIC_H_

#include <avr/io.h>
#include <avr/interrupt.h>
#include "util/delay.h"

//function to disable interrupts
#define AVR_ENTER_CRITICAL_REGION()	cli()

//function to enable interrupts
#define AVR_LEAVE_CRITICAL_REGION()	sei()

void chipSelect(void);
void chipRelease(void);
uint8_t standbyMode(void);
void activeMode(void);
void setMode(uint8_t mode);
void initalizeHardwareForNordic(void);
uint8_t SPI_TransceiveByte(uint8_t data);

#endif /* NORDIC_HARDWARE_SPECIFIC_H_ */
