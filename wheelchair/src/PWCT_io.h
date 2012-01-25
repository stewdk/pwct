/*
 * PWCT_io.h
 *
 *  Created on: Mar 28, 2011
 *      Author: grant
 */

#ifndef PWCT_IO_H_
#define PWCT_IO_H_


void PulsePGDTEstop(void);

void initPWCTio(void);
void StopMove(void);
void Move(uint8_t moveDir);
uint8_t GetMoveDirection(void);
bool LimitSwitchPressed(void);
uint8_t ActuatorSwitchPressed(void);
bool EmergencyStopPressed(void);
void GetInputStates(uint8_t * states);
void SampleInputs(void);

void BumperForwardPressed(void);
void BumperForwardReleased(void);
void BumperReversePressed(void);
void BumperReverseReleased(void);
void BumperLeftPressed(void);
void BumperLeftReleased(void);
void BumperRightPressed(void);
void BumperRightReleased(void);
void ResetBumperThreshold(void);
void PrintBumperStates(void);



#endif /* PWCT_IO_H_ */
