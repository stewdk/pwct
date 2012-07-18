/*
 * PWCT_io.h
 *
 *  Created on: Mar 28, 2011
 *      Author: grant
 */

#ifndef PWCT_IO_H_
#define PWCT_IO_H_

#include "../atmel/avr_compiler.h"

void initPWCTio(void);
void SampleInputs(void);
void OmniStopMove(void);
void OmniMove(uint8_t moveDir);
uint8_t GetMoveDirection(void);
uint8_t getPANEL_BUMPER_OVERRIDE(void);
uint16_t getWiredPropJoySpeed(void);
uint16_t getWiredPropJoyDirection(void);

bool LimitSwitchPressed(void);
uint8_t ActuatorSwitchPressed(void);
bool PanelEStopPressed(void);
void PulsePGDTEstop(void);

#endif /* PWCT_IO_H_ */
