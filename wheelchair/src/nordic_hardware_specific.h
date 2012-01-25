/*
 * nordic_hardware_specific.h
 *
 *  Created on: Apr 12, 2011
 *      Author: grant
 */

#ifndef NORDIC_HARDWARE_SPECIFIC_H_
#define NORDIC_HARDWARE_SPECIFIC_H_

#include "avr_compiler.h"
#include "util.h"

//function to enable interrupts
#define SEI()	sei()

//function to disable interrupts
#define CLI()	cli()

void chipSelect(void);
void chipRelease(void);
uint8_t standbyMode(void);
void activeMode(void);
void setMode(uint8_t mode);
void initalizeHardwareForNordic(void);
uint8_t SPI_TransceiveByte(uint8_t data);

void setInstructorEStop(uint8_t state);
void setInstructorLAUp(uint8_t state);
void setInstructorLADown(uint8_t state);
void setInstructorForward(uint8_t state);
void setInstructorReverse(uint8_t state);
void setInstructorLeft(uint8_t state);
void setInstructorRight(uint8_t state);

//user doesn't need to define this function, but does need to call it from their own interrupt
//triggered by the falling edge of the IRQ pin from the nordic chip
uint8_t nordic_IRQ(void);

//already defined, callable by user
void ClearLastPacket(void);
void SetInstructorRemote(void);

#endif /* NORDIC_HARDWARE_SPECIFIC_H_ */
