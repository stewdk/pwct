/*
 * menu.h
 *
 * Created: 8/16/2012 10:19:43 AM
 *  Author: Stew
 */ 

#ifndef MENU_H_
#define MENU_H_

void menuInit();
void menuUpdate(int16_t speed, int16_t dir);
void menuPlatformDownPushed();
void menuPlatformUpPushed();
uint8_t menuGetIsPlatformDown();
float menuGetFwdThrow(void);
float menuGetRevThrow(void);
float menuGetTurnThrow(void);
uint8_t menuGetTopFwdSpeed(void);
uint8_t menuGetTopRevSpeed(void);
uint8_t menuGetTopTurnSpeed(void);
float menuGetSensitivity(void);
uint8_t menuGetAcceleration(void);
uint8_t menuGetDeceleration(void);
uint8_t menuGetOuterDeadBand(void);
uint8_t menuGetCenterDeadBand(void);
uint8_t menuGetPropAsSwitch(void);
uint8_t menuGetInvert(void);
void incrementWirelessTimeout();

#endif /* MENU_H_ */
