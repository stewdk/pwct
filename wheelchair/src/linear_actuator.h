/*
 * linear_actuator.h
 *
 *  Created on: Mar 26, 2011
 *      Author: grant
 */

#ifndef LINEAR_ACTUATOR_H_
#define LINEAR_ACTUATOR_H_

void initLinearActuators(void);
int8_t RaisePlatform(void);
int8_t LowerPlatform(void);
int8_t StopPlatform(void);

void PrintLACurrents(void);

//int8_t RaisePlatform(uint8_t ratePercent);
//int8_t LowerPlatform(uint8_t ratePercent);

#endif /* LINEAR_ACTUATOR_H_ */
