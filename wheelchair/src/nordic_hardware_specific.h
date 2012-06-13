/*
 * nordic_hardware_specific.h
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */

#ifndef NORDIC_HARDWARE_SPECIFIC_H_
#define NORDIC_HARDWARE_SPECIFIC_H_

#include "../atmel/avr_compiler.h"

void chipSelect(void);
void chipRelease(void);
uint8_t standbyMode(void);
void activeMode(void);
void setMode(uint8_t mode);
void initalizeHardwareForNordic(void);
uint8_t SPI_TransceiveByte(uint8_t data);

#endif /* NORDIC_HARDWARE_SPECIFIC_H_ */
