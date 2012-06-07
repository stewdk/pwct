/*
 * nordic_hardware_specific.h
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */

#ifndef NORDIC_HARDWARE_SPECIFIC_H_
#define NORDIC_HARDWARE_SPECIFIC_H_

#include "../atmel/avr_compiler.h"
#include "util.h"

void chipSelect(void);
void chipRelease(void);
uint8_t standbyMode(void);
void activeMode(void);
void setMode(uint8_t mode);
void initalizeHardwareForNordic(void);
uint8_t SPI_TransceiveByte(uint8_t data);

//The implementations of the following functions are in PWCT_io.c
void setInstructorEStop(uint8_t state);
void setInstructorLAUp(uint8_t state);
void setInstructorLADown(uint8_t state);
void setInstructorForward(uint8_t state);
void setInstructorReverse(uint8_t state);
void setInstructorLeft(uint8_t state);
void setInstructorRight(uint8_t state);

//The implementation of nordic_IRQ() is in nordic_driver.c
//nordic_IRQ() is called from ISR(PORTH_INT0_vect), triggered by the falling edge of the IRQ pin from the nordic chip
uint8_t nordic_IRQ(void);

//Implementations of ClearLastPacket and SetInstructorRemote are in nordic_driver.c
void ClearLastPacket(void);
void SetInstructorRemote(void);

#endif /* NORDIC_HARDWARE_SPECIFIC_H_ */
