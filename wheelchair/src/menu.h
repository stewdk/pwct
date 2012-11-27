/*
 * menu.h
 *
 * Created: 8/16/2012 10:19:43 AM
 *  Author: Stew
 */ 

#ifndef MENU_H_
#define MENU_H_

void menuInit();
uint8_t menuGetMotorsDisabled();
void menuUpdate(int16_t speed, int16_t dir);
void menuPlatformDownPushed();
void menuPlatformUpPushed();
uint8_t menuGetIsPlatformDown();
float menuGetFwdThrow(uint8_t overridden);
float menuGetRevThrow(uint8_t overridden);
float menuGetTurnThrow(uint8_t overridden);
uint8_t menuGetTopFwdSpeed(uint8_t overridden);
uint8_t menuGetTopRevSpeed(uint8_t overridden);
uint8_t menuGetTopTurnSpeed(uint8_t overridden);
double menuGetSensitivity(uint8_t overridden);
uint8_t menuGetAcceleration(uint8_t overridden);
uint8_t menuGetDeceleration(uint8_t overridden);
uint8_t menuGetOuterDeadBand(uint8_t overridden);
uint8_t menuGetCenterDeadBand(uint8_t overridden);
uint8_t menuGetPropAsSwitch(uint8_t overridden);
uint8_t menuGetInvert(uint8_t overridden);
void incrementWirelessTimeout();

#endif /* MENU_H_ */
