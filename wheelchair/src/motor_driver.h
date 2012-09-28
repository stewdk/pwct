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

// Sabertooth Commands
#define MOTOR_CMD_DRIVE_FORWARD_MOTOR_1 0
#define MOTOR_CMD_DRIVE_BACKWARDS_MOTOR_1 1
#define MOTOR_CMD_MIN_VOLTAGE 2
#define MOTOR_CMD_MAX_VOLTAGE 3
#define MOTOR_CMD_DRIVE_FORWARD_MOTOR_2 4
#define MOTOR_CMD_DRIVE_BACKWARDS_MOTOR_2 5
#define MOTOR_CMD_DRIVE_MOTOR_1_7_BIT 6
#define MOTOR_CMD_DRIVE_MOTOR_2_7_BIT 7
#define MOTOR_CMD_DRIVE_FORWARD_MIXED_MODE 8
#define MOTOR_CMD_DRIVE_BACKWARDS_MIXED_MODE 9
#define MOTOR_CMD_TURN_RIGHT_MIXED_MODE 10
#define MOTOR_CMD_TURN_LEFT_MIXED_MODE 11
#define MOTOR_CMD_DRIVE_FORWARDS_BACK_7_BIT 12
#define MOTOR_CMD_TURN_7_BIT 13
#define MOTOR_CMD_SERIAL_TIMEOUT 14
#define MOTOR_CMD_BAUD_RATE 15
#define MOTOR_CMD_RAMPING 16
#define MOTOR_CMD_DEADBAND 17

void initMotorDriver(void);
void motorEStop(void);
void resetMotorEStop(void);
void sendMotorCommand(uint8_t command, uint8_t data);
void setMotors(int16_t speed, int16_t dir);

#endif /* MOTOR_DRIVER_H_ */
