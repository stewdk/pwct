/*
 * test.h
 *
 * Created: 6/8/2012 10:22:18 AM
 *  Author: Stew
 */ 


#ifndef TEST_H_
#define TEST_H_

#include "util.h"

void testJoystickDriveMotors(void);
void testPropJoy(void);
void getStateStr(states state, char *str);
void testNordicWireless(void);
void testInputs(void);
void testMotorDriver(void);

void hardWireControls(void); // Warning: possibly obsolete, untested test
void testBumpers(void); // Warning: definitely obsolete

#endif /* TEST_H_ */