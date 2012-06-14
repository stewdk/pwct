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
void GetInputStates(uint8_t * states);
void SampleInputs(void);
void StopMove(void);
void Move(uint8_t moveDir);
uint8_t GetMoveDirection(void);
uint8_t getPANEL_BUMPER_OVERRIDE(void);
uint16_t getWiredPropJoySpeed(void);
uint16_t getWiredPropJoyDirection(void);

void setInstructorEStop(uint8_t state);
void setInstructorLAUp(uint8_t state);
void setInstructorLADown(uint8_t state);
void setInstructorForward(uint8_t state);
void setInstructorReverse(uint8_t state);
void setInstructorLeft(uint8_t state);
void setInstructorRight(uint8_t state);

void PrintBumperStates(void);
void BumperForwardPressed(void);
void BumperForwardReleased(void);
void BumperReversePressed(void);
void BumperReverseReleased(void);
void BumperLeftPressed(void);
void BumperLeftReleased(void);
void BumperRightPressed(void);
void BumperRightReleased(void);

bool LimitSwitchPressed(void);
uint8_t ActuatorSwitchPressed(void);
bool EmergencyStopPressed(void);
void PulsePGDTEstop(void);

#endif /* PWCT_IO_H_ */
