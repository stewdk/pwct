/*
 * joystick_algorithm.h
 *
 * Created: 9/27/2012 9:52:55 PM
 *  Author: Stew
 */ 

#ifndef JOYSTICK_ALGORITHM_H_
#define JOYSTICK_ALGORITHM_H_

void joystickAlgorithmInit();
void getProportionalMoveDirection(int16_t *speed, int16_t *dir);

#endif /* JOYSTICK_ALGORITHM_H_ */
