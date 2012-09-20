/*
 * menu.h
 *
 * Created: 8/16/2012 10:19:43 AM
 *  Author: Stew
 */ 

#ifndef MENU_H_
#define MENU_H_

void menuInit(void);
void menuUpdate(int16_t speed, int16_t dir);
float menuGetFwdThrow(void);
float menuGetRevThrow(void);
float menuGetTurnThrow(void);
uint8_t menuGetTopFwdSpeed(void);
uint8_t menuGetTopRevSpeed(void);
uint8_t menuGetTopTurnSpeed(void);
uint8_t menuGetCenterDeadBand(void);
uint8_t menuGetAcceleration(void);
uint8_t menuGetPropAsSwitch(void);

#endif /* MENU_H_ */
