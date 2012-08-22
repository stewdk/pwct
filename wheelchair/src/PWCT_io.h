/*
 * PWCT_io.h
 *
 *  Created on: Mar 28, 2011
 *      Author: grant
 */

#ifndef PWCT_IO_H_
#define PWCT_IO_H_

#include <avr/io.h>
#include <stdint.h>
#include <stdbool.h>

typedef struct {
	PORT_t *port;
	uint8_t pin_bm;
	volatile uint8_t previous_values;
	volatile uint8_t debounced_value;
	uint8_t previous_debounced_value;
} debounced_input;

void initPWCTio(void);
void SampleInputs(void);
void OmniStopMove(void);
void OmniMove(uint8_t moveDir);
uint8_t getSwitchMoveDirection(void);
uint8_t getPANEL_BUMPER_OVERRIDE(void);
uint16_t getWiredPropJoySpeed(void);
uint16_t getWiredPropJoyDirection(void);
uint8_t lcdUpFallingEdge(void);
uint8_t lcdDownFallingEdge(void);
uint8_t lcdRightFallingEdge(void);
uint8_t lcdLeftFallingEdge(void);

bool LimitSwitchPressed(void);
uint8_t ActuatorSwitchPressed(void);
bool PanelEStopPressed(void);
void PulsePGDTEstop(void);

#endif /* PWCT_IO_H_ */
