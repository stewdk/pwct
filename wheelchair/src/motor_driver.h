/*
 * motor_driver.h
 *
 * Created: 6/4/2012 3:30:02 PM
 *  Author: Stew
 */ 


#ifndef MOTOR_DRIVER_H_
#define MOTOR_DRIVER_H_

typedef union {
	uint8_t array[4];
	struct {
		uint8_t address;
		uint8_t command;
		uint8_t data;
		uint8_t checksum;
	} parts;
} sabertooth_packet;

#define SABERTOOTH_ADDRESS 128

void initMotorDriver(void);
void motorEStop(void);
void resetMotorEStop(void);
void sendMotorPacket(sabertooth_packet *packet);
void sendMotorCommand(uint8_t command, uint8_t data);

#endif /* MOTOR_DRIVER_H_ */
