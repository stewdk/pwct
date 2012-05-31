/*
 * PWCT_io.c
 *
 *  Created on: Mar 28, 2011
 *      Author: grant
 */
#include "../atmel/avr_compiler.h"
#include "../atmel/TC_driver.h"
#include "../atmel/port_driver.h"
#include "../atmel/pmic_driver.h"
#include "nordic_driver.h"
#include "util/delay.h"
#include "util.h"
#include "PWCT_io.h"
#include "util/delay.h"
#include "stdio.h"

//#include "nordic_hardware_specific.h"

//debounce timers 1tick = 8us
//1250 ticks = 10ms
#define DEBOUNCE_PERIOD 1250

//pulse timer 1tick = 8us
//31250 ticks = 250ms
#define PGDT_ESTOP_PULSE_PERIOD 31250

#define DEBOUNCE_BUTTONS	0
#define ENABLE_INSTRUCTOR_REMOTE_LINEAR_ACTUATOR_CONTROL	0

/* Input/Output List:
 *	Bumper Front										Filter
 *	Bumper Left											Filter
 *	Bumper Right										Filter
 *	Bumper Back											Filter
 *	Instructor Forward											Clean
 *	Instructor Reverse											Clean
 *	Instructor Left												Clean
 *	Instructor Right											Clean
 *	Instructor LA Up											Clean
 *	Instructor LA Down											Clean
 *	Instructor Disable Student Joy								Clean
 *	Student Forward					PJ3		Debounce
 *	Student Reverse					PJ4		Debounce
 *	Student Left					PJ5		Debounce
 *	Student Right					PJ6		Debounce
 *	Student Fifth					PJ7		Debounce
 *	Buddy Button Forward			PH6		Debounce
 *	Buddy Button Reverse			PH7		Debounce
 *	Buddy Button Left				PJ0		Debounce
 *	Buddy Button Right				PJ1		Debounce
 *	Buddy Button Fifth				PJ2		Debounce
 *	Emergency Stop					PK2		Debounce
 *	Omni+ On/Off Switch				PK6									Output
 *	Panel LA Up						PK0		Debounce
 *	Panel LA Down					PK3		Debounce
 *	Panel LA LED					PK5									Output
 *	Panel Bumper Override LED		PK4									Output
 *	Panel Bumper Override Switch	PK1		Debounce
 *	Limit Switch					PK7		Debounce
 *	Out Forward						PH1									Output
 *	Out Reverse						PH0									Output
 *	Out Left						PH3									Output
 *	Out Right						PH4									Output
 *	Out Fifth						PH5									Output
 */

//these flags are all active low
static uint8_t 	INSTRUCTOR_FORWARD, INSTRUCTOR_REVERSE, INSTRUCTOR_LEFT, INSTRUCTOR_RIGHT,
				INSTRUCTOR_LA_UP, INSTRUCTOR_LA_DOWN,
				INSTRUCTOR_ESTOP,
				BB_FORWARD, BB_REVERSE, BB_LEFT, BB_RIGHT, BB_FIFTH,
				STUDENT_FORWARD, STUDENT_REVERSE, STUDENT_LEFT, STUDENT_RIGHT, STUDENT_FIFTH,
				ESTOP,
				PANEL_LA_UP, PANEL_LA_DOWN,
				PANEL_BUMPER_OVERRIDE,
				BUMPER_FORWARD = 1, BUMPER_REVERSE = 1, BUMPER_LEFT = 1, BUMPER_RIGHT = 1,
				LIMIT_SWITCH,
				INVERT_SWITCH;

void PulsePGDTEstop(void);

void GetInputStates(uint8_t * states)
{
//	printf("|      Remote          |                Panel               |Limit|             Bumpers              |\r\n");
//	printf("| Joystck | LA  | Estp | Buddy Btn | Joystick  | Estp |LA  | OvrRd|     | 1    2    3    4    5    6    7  |\r\n");
//	printf("| U D L R | U D |      | U D L R S | U D L R S |      |U D |      |     |                                  |\r\n");

	*states++ = INSTRUCTOR_FORWARD;
	*states++ = INSTRUCTOR_REVERSE;
	*states++ = INSTRUCTOR_LEFT;
	*states++ = INSTRUCTOR_RIGHT;
	*states++ = INSTRUCTOR_LA_UP;
	*states++ = INSTRUCTOR_LA_DOWN;
	*states++ = INSTRUCTOR_ESTOP;
	*states++ = BB_FORWARD;
	*states++ = BB_REVERSE;
	*states++ = BB_LEFT;
	*states++ = BB_RIGHT;
	*states++ = BB_FIFTH;
	*states++ = STUDENT_FORWARD;
	*states++ = STUDENT_REVERSE;
	*states++ = STUDENT_LEFT;
	*states++ = STUDENT_RIGHT;
	*states++ = STUDENT_FIFTH;
	*states++ = ESTOP;
	*states++ = PANEL_LA_UP;
	*states++ = PANEL_LA_DOWN;
	*states++ = PANEL_BUMPER_OVERRIDE;
	*states++ = LIMIT_SWITCH;
	*states++ = 0;
	*states++ = 0;
	*states++ = 0;
	*states++ = 0;
	*states++ = 0;
	*states++ = 0;
	*states++ = 0;

}

void initPWCTio(void)
{
	//turn off timers
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_OFF_gc );
	TC1_ConfigClockSource( &TCC1, TC_CLKSEL_OFF_gc );
	TC0_ConfigClockSource( &TCD0, TC_CLKSEL_OFF_gc );
	TC1_ConfigClockSource( &TCD1, TC_CLKSEL_OFF_gc );
	TC1_ConfigClockSource( &TCE1, TC_CLKSEL_OFF_gc );
	TC1_ConfigClockSource( &TCF1, TC_CLKSEL_OFF_gc );

	// Set the TC period.
	TC_SetPeriod( &TCC0, 0xFFFF );
	TC_SetPeriod( &TCC1, 0xFFFF );
	TC_SetPeriod( &TCD0, 0xFFFF );
	TC_SetPeriod( &TCD1, 0xFFFF );
	TC_SetPeriod( &TCE1, 0xFFFF );
	TC_SetPeriod( &TCF1, 0xFFFF );

	//Set timer in normal mode
	TC0_ConfigWGM( &TCC0, TC_WGMODE_NORMAL_gc );
	TC1_ConfigWGM( &TCC1, TC_WGMODE_NORMAL_gc );
	TC0_ConfigWGM( &TCD0, TC_WGMODE_NORMAL_gc );
	TC1_ConfigWGM( &TCD1, TC_WGMODE_NORMAL_gc );
	TC1_ConfigWGM( &TCE1, TC_WGMODE_NORMAL_gc );
	TC1_ConfigWGM( &TCF1, TC_WGMODE_NORMAL_gc );

	#if DEBOUNCE_BUTTONS
	//set up compare interrupts
	TC0_SetCCAIntLevel(&TCC0, TC_CCAINTLVL_MED_gc);
	TC0_SetCCBIntLevel(&TCC0, TC_CCBINTLVL_MED_gc);
	TC0_SetCCCIntLevel(&TCC0, TC_CCCINTLVL_MED_gc);
	TC0_SetCCDIntLevel(&TCC0, TC_CCDINTLVL_MED_gc);
	TC1_SetCCAIntLevel(&TCC1, TC_CCAINTLVL_MED_gc);
	TC1_SetCCBIntLevel(&TCC1, TC_CCBINTLVL_MED_gc);
	TC0_SetCCAIntLevel(&TCD0, TC_CCAINTLVL_MED_gc);
	TC0_SetCCBIntLevel(&TCD0, TC_CCBINTLVL_MED_gc);
	TC0_SetCCCIntLevel(&TCD0, TC_CCCINTLVL_MED_gc);
	TC0_SetCCDIntLevel(&TCD0, TC_CCDINTLVL_MED_gc);
	TC1_SetCCAIntLevel(&TCD1, TC_CCAINTLVL_MED_gc);
	TC1_SetCCBIntLevel(&TCD1, TC_CCBINTLVL_MED_gc);
	TC1_SetCCAIntLevel(&TCE1, TC_CCAINTLVL_MED_gc);
	TC1_SetCCBIntLevel(&TCE1, TC_CCBINTLVL_MED_gc);
	TC1_SetCCAIntLevel(&TCF1, TC_CCAINTLVL_MED_gc);
#endif
	TC1_SetCCBIntLevel(&TCF1, TC_CCBINTLVL_MED_gc);

	//start clocks
	TC0_ConfigClockSource( &TCC0, TC_CLKSEL_DIV256_gc );
	TC1_ConfigClockSource( &TCC1, TC_CLKSEL_DIV256_gc );
	TC0_ConfigClockSource( &TCD0, TC_CLKSEL_DIV256_gc );
	TC1_ConfigClockSource( &TCD1, TC_CLKSEL_DIV256_gc );
	TC1_ConfigClockSource( &TCE1, TC_CLKSEL_DIV256_gc );
	TC1_ConfigClockSource( &TCF1, TC_CLKSEL_DIV256_gc );

	PORT_ConfigurePins( &PORTC, PIN1_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc );
	PORT_ConfigurePins( &PORTC, PIN2_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc );
	PORT_ConfigurePins( &PORTH, PIN6_bm | PIN7_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_BOTHEDGES_gc );
	PORT_ConfigurePins( &PORTJ, 0xFF, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_BOTHEDGES_gc );
	PORT_ConfigurePins( &PORTK, PIN0_bm | PIN1_bm | PIN3_bm | PIN7_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_BOTHEDGES_gc );
	PORT_ConfigurePins( &PORTK, PIN2_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc );
	PORT_ConfigurePins( &PORTD, PIN7_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc );
	PORT_ConfigurePins( &PORTE, PIN4_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc );
	PORT_ConfigurePins( &PORTE, PIN5_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc );
	PORT_SetPinsAsInput( &PORTC, PIN1_bm);
	PORT_SetPinsAsInput( &PORTC, PIN2_bm);
	PORT_SetPinsAsInput( &PORTH, PIN6_bm | PIN7_bm);
	PORT_SetPinsAsInput( &PORTJ, PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN5_bm | PIN7_bm );
	PORT_SetPinsAsInput( &PORTD, PIN7_bm);
	PORT_SetPinsAsInput( &PORTE, PIN4_bm);
	PORT_SetPinsAsInput( &PORTE, PIN5_bm);
	PORT_SetPinsAsInput( &PORTK,  PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN7_bm);
#if DEBOUNCE_BUTTONS
	//set up pin change interrupts
	PORT_ConfigureInterrupt1( &PORTH, PORT_INT1LVL_MED_gc, PIN6_bm | PIN7_bm);
	PORT_ConfigureInterrupt0( &PORTJ, PORT_INT0LVL_MED_gc, PIN0_bm | PIN1_bm | PIN2_bm | PIN7_bm);
	PORT_ConfigureInterrupt1( &PORTJ, PORT_INT1LVL_MED_gc, PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm);
	PORT_ConfigureInterrupt1( &PORTK, PORT_INT1LVL_MED_gc, PIN1_bm | PIN7_bm);
	PORT_ConfigureInterrupt0( &PORTK, PORT_INT0LVL_MED_gc, PIN0_bm | PIN2_bm | PIN3_bm);
	PORT_ConfigureInterrupt1( &PORTK, PORT_INT1LVL_MED_gc, PIN1_bm | PIN7_bm);
#endif

	//set outputs
	PORTK.OUTCLR = PIN4_bm | PIN5_bm;	//leds off
	PORTK.OUTCLR = PIN6_bm;				//switch disabled
	PORTK.DIRSET = PIN4_bm | PIN5_bm | PIN6_bm;
	PORTH.OUTCLR = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm;	//switch disabled
	PORTH.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm;

	//get default values
	SampleInputs();

	//set default values
	INSTRUCTOR_FORWARD = 0;
	INSTRUCTOR_REVERSE = 0;
	INSTRUCTOR_LEFT = 0;
	INSTRUCTOR_RIGHT = 0;
	INSTRUCTOR_LA_UP = 0;
	INSTRUCTOR_LA_DOWN = 0;
	INSTRUCTOR_ESTOP = 0;

	PMIC_EnableMediumLevel();
}

void SampleInputs(void)
{
	static uint8_t estopDebounceFlg = 0;
	static uint8_t EstopPulseSent = 0;
	uint8_t panelBumperOverrideOld;

	panelBumperOverrideOld = PANEL_BUMPER_OVERRIDE;

	INVERT_SWITCH			= PORTC.IN & PIN2_bm;
	BB_FORWARD				= PORTE.IN & PIN5_bm;
	BB_REVERSE				= PORTC.IN & PIN1_bm;
	BB_LEFT					= PORTJ.IN & PIN0_bm;
	BB_RIGHT				= PORTJ.IN & PIN1_bm;
	BB_FIFTH				= PORTJ.IN & PIN2_bm;
	STUDENT_FORWARD			= PORTJ.IN & PIN3_bm;
	STUDENT_REVERSE			= PORTE.IN & PIN4_bm;
	STUDENT_LEFT			= PORTJ.IN & PIN5_bm;
	STUDENT_RIGHT			= PORTD.IN & PIN7_bm;
	STUDENT_FIFTH			= PORTJ.IN & PIN7_bm;
	PANEL_LA_UP				= PORTK.IN & PIN0_bm;
	PANEL_BUMPER_OVERRIDE	= PORTK.IN & PIN1_bm;
	ESTOP					= PORTK.IN & PIN2_bm;
	PANEL_LA_DOWN			= PORTK.IN & PIN3_bm;
	LIMIT_SWITCH			= PORTK.IN & PIN7_bm;

	//printf("\rSTUDENT_FORWARD = %d  BB_FORWARD = %d  STUDENT_REVERSE = %d  BB_REVERSE = %d", STUDENT_FORWARD, BB_FORWARD, STUDENT_REVERSE, BB_REVERSE);

	if(ESTOP) {
		estopDebounceFlg = 0;
		EstopPulseSent = 0;
	}
	else if(!EstopPulseSent) {
		estopDebounceFlg++;
		if(estopDebounceFlg >= 3) {
			PulsePGDTEstop();
			EstopPulseSent = 1;
		}
	}

	//set bumper override led led
	if(PANEL_BUMPER_OVERRIDE) {
		PORTK.OUTCLR = PIN4_bm;
	} else {
		PORTK.OUTSET = PIN4_bm;
	}

	//check if panel bumper override was toggled
	if(!PANEL_BUMPER_OVERRIDE) {
		ResetBumperThreshold();
	}
}

void StopMove(void)
{
	PORTH.OUTCLR = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm;
}

//moveDir is a bit field of active directions 0b000SFVLR
//S = Select/Fifth button
//F = Forward
//V = Reverse
//L = Left
//R = Right
void Move(uint8_t moveDir)
{
	/*	Out Forward		PH1
	 *	Out Reverse		PH0
	 *	Out Left		PH3
	 *	Out Right		PH4
	 *	Out Fifth		PH5
	 */

	if(moveDir & 0x10) {			//fifth button
		PORTH.OUTSET = PIN5_bm;
	}
	else {
		PORTH.OUTCLR = PIN5_bm;
	}


	if((moveDir & 0x0C) == 0x08) {	//forward
		PORTH.OUTSET = PIN1_bm;
	}
	else {
		PORTH.OUTCLR = PIN1_bm;
	}

	if((moveDir & 0x0C) == 0x04) {	//reverse
		PORTH.OUTSET = PIN0_bm;
	}
	else {
		PORTH.OUTCLR = PIN0_bm;
	}

	if((moveDir & 0x03) == 0x02) {	//left
		PORTH.OUTSET = PIN4_bm;
	}
	else {
		PORTH.OUTCLR = PIN4_bm;
	}

	if((moveDir & 0x03) == 0x01) {	//right
		PORTH.OUTSET = PIN3_bm;
	}
	else {
		PORTH.OUTCLR = PIN3_bm;
	}
}

//returns bit field of active directions 0b000SFVLR
//S = Select/Fifth button
//F = Forward
//V = Reverse
//L = Left
//R = Right
uint8_t GetMoveDirection(void)
{
	uint8_t moveDir = 0;

	//fifth switch
	if( !BB_FIFTH || !STUDENT_FIFTH ) {	//if buddy button fifth or student fifth selected
		moveDir |= 0x10;
	}

	//Instructor forward and reverse
	if(INSTRUCTOR_FORWARD && !INSTRUCTOR_REVERSE) {	//if instructor forward selected and not instructor reverse selected
		moveDir |= 0x08;
	}
	else if(INSTRUCTOR_REVERSE && !INSTRUCTOR_FORWARD) {	//if instructor reverse selected and not instructor forward selected
		moveDir |= 0x04;
	}
	//Instructor left and right
	if(INSTRUCTOR_LEFT && !INSTRUCTOR_RIGHT) {	//if instructor forward selected and not instructor reverse selected
		moveDir |= 0x02;
	}
	else if(INSTRUCTOR_RIGHT && !INSTRUCTOR_LEFT) {	//if instructor reverse selected and not instructor forward selected
		moveDir |= 0x01;
	}

	if((moveDir & 0x0F) == 0) {	//if the instructor hasn't already picked a movement
		//student forward and reverse
		if((!STUDENT_FORWARD || !BB_FORWARD) && (STUDENT_REVERSE && BB_REVERSE) ) {	//if student or bb forward selected and (student and bb reverse not selected)
			//if(!PANEL_BUMPER_OVERRIDE || BUMPER_FORWARD) {	//if bumper override switch selected or forward bumper not activated
				if(INVERT_SWITCH){
					moveDir |= 0x04;
				}
				else{
					moveDir |= 0x08;
				}
			//}
		}
		else if( (!STUDENT_REVERSE || !BB_REVERSE) && (STUDENT_FORWARD && BB_FORWARD) ) {	//if student or bb reverse selected and (student and bb forward not selected)
			//if(!PANEL_BUMPER_OVERRIDE || BUMPER_REVERSE) {	//if bumper override switch selected or reverse bumper not activated
			if(INVERT_SWITCH){
				moveDir |= 0x08;
			}
			else{
				moveDir |= 0x04;
			}
			//}
		}

		//student left and right
		if( (!STUDENT_LEFT || !BB_LEFT) && (STUDENT_RIGHT && BB_RIGHT) ) {	//if student or bb forward selected and (student and bb reverse not selected)
			//if(!PANEL_BUMPER_OVERRIDE || BUMPER_LEFT) {	//if bumper override switch selected or forward bumper not activated
				moveDir |= 0x02;
			//}
		}
		else if( (!STUDENT_RIGHT || !BB_RIGHT) && (STUDENT_LEFT && BB_LEFT) ) {	//if student or bb reverse selected and (student and bb forward not selected)
			//if(!PANEL_BUMPER_OVERRIDE || BUMPER_RIGHT) {	//if bumper override switch selected or reverse bumper not activated
				moveDir |= 0x01;
			//}
		}
	}

	return moveDir;
}
void setInstructorEStop(uint8_t state)
{
	//make sure to only send one pulse
	if((INSTRUCTOR_ESTOP != state) && (state == 0)) {
		PulsePGDTEstop();
	}
	INSTRUCTOR_ESTOP = state;
}
void setInstructorLAUp(uint8_t state)
{
	INSTRUCTOR_LA_UP = state;
}
void setInstructorLADown(uint8_t state)
{
	INSTRUCTOR_LA_DOWN = state;
}
void setInstructorForward(uint8_t state)
{
	INSTRUCTOR_FORWARD = state;
}
void setInstructorReverse(uint8_t state)
{
	INSTRUCTOR_REVERSE = state;
}
void setInstructorLeft(uint8_t state)
{
	INSTRUCTOR_LEFT = state;
}
void setInstructorRight(uint8_t state)
{
	INSTRUCTOR_RIGHT = state;
}

void PrintBumperStates(void)
{
	printf("Rv%d, Fw%d, Lt%d, Rt%d\n\r", BUMPER_REVERSE, BUMPER_FORWARD, BUMPER_LEFT, BUMPER_RIGHT);
}

void BumperForwardPressed(void)
{
	BUMPER_FORWARD = 0;
}

void BumperForwardReleased(void)
{
	BUMPER_FORWARD = 1;
}

void BumperReversePressed(void)
{
	BUMPER_REVERSE = 0;
}

void BumperReverseReleased(void)
{
	BUMPER_REVERSE = 1;
}
void BumperLeftPressed(void)
{
	BUMPER_LEFT = 0;
}

void BumperLeftReleased(void)
{
	BUMPER_LEFT = 1;
}
void BumperRightPressed(void)
{
	BUMPER_RIGHT = 0;
}

void BumperRightReleased(void)
{
	BUMPER_RIGHT = 1;
}

bool LimitSwitchPressed(void)
{
	return !LIMIT_SWITCH;
}

/* 0 = NO
 * 1 = DOWN
 * 2 = UP
 */
uint8_t ActuatorSwitchPressed(void)
{
	//panel controls take priority over instructor remote controls
	if(!PANEL_LA_UP && PANEL_LA_DOWN) {
		return 2;	//up is pressed
	}
	else if(PANEL_LA_UP && !PANEL_LA_DOWN) {
		return 1;	//down is pressed
	}
#if ENABLE_INSTRUCTOR_REMOTE_LINEAR_ACTUATOR_CONTROL
	else if(INSTRUCTOR_LA_UP && !INSTRUCTOR_LA_DOWN) {
		return 2;	//up is pressed
	}
	else if(!INSTRUCTOR_LA_UP && INSTRUCTOR_LA_DOWN) {
		return 1;	//down is pressed
	}
#endif
	else {
		return 0;	//nothing is pressed or up and down are both pressed
	}
}

bool EmergencyStopPressed(void)
{
	return !ESTOP;
}

/*******************************************************************************
 * Debounce Interrupts
 *******************************************************************************/
ISR(PORTH_INT1_vect)
{
	uint8_t switchedPins = 0;

	//BB_FORWARD				= PORTH.IN & PIN6_bm;
	//BB_REVERSE				= PORTH.IN & PIN7_bm;

	switchedPins = (BB_FORWARD | BB_REVERSE) ^ ((PIN6_bm | PIN7_bm) & PORTH.IN);

	if((switchedPins & PIN6_bm) && !(TCC1.CTRLB & TC1_CCAEN_bm)) {	//BB_FORWARD
		//start debounce timer
		TC_SetCompareA(&TCC1, TCC1.CNT + DEBOUNCE_PERIOD);
		TC1_EnableCCChannels( &TCC1, TC1_CCAEN_bm);
	}
	if((switchedPins & PIN7_bm) && !(TCC1.CTRLB & TC1_CCBEN_bm)) {	//BB_REVERSE
		//start debounce timer
		TC_SetCompareB(&TCC1, TCC1.CNT + DEBOUNCE_PERIOD);
		TC1_EnableCCChannels( &TCC1, TC1_CCBEN_bm);
	}
}

ISR(PORTJ_INT0_vect)
{
	uint8_t switchedPins = 0;

	//BB_LEFT			PJ0
	//BB_RIGHT			PJ1
	//BB_FIFTH			PJ2
	//STUDENT_FIFTH		PJ7
	switchedPins = (BB_LEFT | BB_RIGHT | BB_FIFTH | STUDENT_FIFTH) ^ ((PIN0_bm | PIN1_bm | PIN2_bm | PIN7_bm) & PORTJ.IN);

	if((switchedPins & PIN0_bm) && !(TCC0.CTRLB & TC0_CCAEN_bm)) {	//BB_LEFT
		//start debounce timer
		TC_SetCompareA(&TCC0, TCC0.CNT + DEBOUNCE_PERIOD);
		TC0_EnableCCChannels( &TCC0, TC0_CCAEN_bm);
	}
	if((switchedPins & PIN1_bm) && !(TCC0.CTRLB & TC0_CCBEN_bm)) {	//BB_RIGHT
		//start debounce timer
		TC_SetCompareB(&TCC0, TCC0.CNT + DEBOUNCE_PERIOD);
		TC0_EnableCCChannels( &TCC0, TC0_CCBEN_bm);
	}
	if((switchedPins & PIN2_bm) && !(TCC0.CTRLB & TC0_CCCEN_bm)) {	//BB_FIFTH
		//start debounce timer
		TC_SetCompareC(&TCC0, TCC0.CNT + DEBOUNCE_PERIOD);
		TC0_EnableCCChannels( &TCC0, TC0_CCCEN_bm);
	}
	if((switchedPins & PIN7_bm) && !(TCC0.CTRLB & TC0_CCDEN_bm)) {	//STUDENT_FIFTH
		//start debounce timer
		TC_SetCompareD(&TCC0, TCC0.CNT + DEBOUNCE_PERIOD);
		TC0_EnableCCChannels( &TCC0, TC0_CCDEN_bm);
	}
}

ISR(PORTJ_INT1_vect)
{
	uint8_t switchedPins = 0;

	//STUDENT_FORWARD	PJ3
	//STUDENT_REVERSE	PJ4
	//STUDENT_LEFT		PJ5
	//STUDENT_RIGHT		PJ6
	switchedPins = (STUDENT_FORWARD | STUDENT_REVERSE | STUDENT_LEFT | STUDENT_RIGHT) ^ ((PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm) & PORTJ.IN);

	if((switchedPins & PIN3_bm) && !(TCD0.CTRLB & TC0_CCAEN_bm)) {	//STUDENT_FORWARD
		//start debounce timer
		TC_SetCompareA(&TCD0, TCD0.CNT + DEBOUNCE_PERIOD);
		TC0_EnableCCChannels( &TCD0, TC0_CCAEN_bm);
	}
	if((switchedPins & PIN4_bm) && !(TCD0.CTRLB & TC0_CCBEN_bm)) {	//STUDENT_REVERSE
		//start debounce timer
		TC_SetCompareB(&TCD0, TCD0.CNT + DEBOUNCE_PERIOD);
		TC0_EnableCCChannels( &TCD0, TC0_CCBEN_bm);
	}
	if((switchedPins & PIN5_bm) && !(TCD0.CTRLB & TC0_CCCEN_bm)) {	//STUDENT_LEFT
		//start debounce timer
		TC_SetCompareC(&TCD0, TCD0.CNT + DEBOUNCE_PERIOD);
		TC0_EnableCCChannels( &TCD0, TC0_CCCEN_bm);
	}
	if((switchedPins & PIN6_bm) && !(TCD0.CTRLB & TC0_CCDEN_bm)) {	//STUDENT_RIGHT
		//start debounce timer
		TC_SetCompareD(&TCD0, TCD0.CNT + DEBOUNCE_PERIOD);
		TC0_EnableCCChannels( &TCD0, TC0_CCDEN_bm);
	}
}

ISR(PORTK_INT0_vect)
{
	uint8_t switchedPins = 0;

	//PANEL_LA_UP		PK0
	//ESTOP				PK2
	//PANEL_LA_DOWN		PK3

	switchedPins = (PANEL_LA_UP | ESTOP | PANEL_LA_DOWN) ^ ((PIN0_bm | PIN2_bm | PIN3_bm) & PORTK.IN);

	if((switchedPins & PIN0_bm) && !(TCE1.CTRLB & TC1_CCAEN_bm)) {	//PANEL_LA_UP
		//start debounce timer
		TC_SetCompareA(&TCE1, TCE1.CNT + DEBOUNCE_PERIOD);
		TC1_EnableCCChannels( &TCE1, TC1_CCAEN_bm);
	}
	if((switchedPins & PIN2_bm) && !(TCE1.CTRLB & TC1_CCBEN_bm)) {	//ESTOP
		//start debounce timer
		TC_SetCompareB(&TCE1, TCE1.CNT + DEBOUNCE_PERIOD);
		TC1_EnableCCChannels( &TCE1, TC1_CCBEN_bm);
	}
	if((switchedPins & PIN3_bm) && !(TCF1.CTRLB & TC1_CCAEN_bm)) {	//PANEL_LA_DOWN
		//start debounce timer
		TC_SetCompareA(&TCF1, TCF1.CNT + DEBOUNCE_PERIOD);
		TC1_EnableCCChannels( &TCF1, TC1_CCAEN_bm);
	}
}

ISR(PORTK_INT1_vect)
{
	uint8_t switchedPins = 0;

	//PANEL_BUMPER_OVERRIDE		PK1
	//LIMIT_SWITCH				PK7

	switchedPins = (PANEL_BUMPER_OVERRIDE | LIMIT_SWITCH) ^ ((PIN1_bm | PIN7_bm)  & PORTK.IN);

	if((switchedPins & PIN1_bm) && !(TCD1.CTRLB & TC1_CCAEN_bm)) {	//PANEL_BUMPER_OVERRIDE
		//start debounce timer
		TC_SetCompareA(&TCD1, TCD1.CNT + DEBOUNCE_PERIOD);
		TC1_EnableCCChannels( &TCD1, TC1_CCAEN_bm);
	}
	if((switchedPins & PIN7_bm) && !(TCD1.CTRLB & TC1_CCBEN_bm)) {	//LIMIT_SWITCH
		//start debounce timer
		TC_SetCompareB(&TCD1, TCD1.CNT + DEBOUNCE_PERIOD);
		TC1_EnableCCChannels( &TCD1, TC1_CCBEN_bm);
	}
}

//after a debouncing period set the variable to the pin value
ISR(TCC0_CCA_vect)	//BB_LEFT
{
	if((TCC0.CTRLB & TC0_CCAEN_bm) == 0) {
		return;
	}
	TC0_DisableCCChannels( &TCC0, TC0_CCAEN_bm);
	BB_LEFT	= PORTJ.IN & PIN0_bm;
}

ISR(TCC0_CCB_vect)	//BB_RIGHT
{
	if((TCC0.CTRLB & TC0_CCBEN_bm) == 0) {
		return;
	}
	TC0_DisableCCChannels( &TCC0, TC0_CCBEN_bm);
	BB_RIGHT	= PORTJ.IN & PIN1_bm;
}

ISR(TCC0_CCC_vect)	//BB_FIFTH
{
	if((TCC0.CTRLB & TC0_CCCEN_bm) == 0) {
		return;
	}
	TC0_DisableCCChannels( &TCC0, TC0_CCCEN_bm);
	BB_FIFTH	= PORTJ.IN & PIN2_bm;
}

ISR(TCC0_CCD_vect)	//STUDENT_FIFTH
{
	if((TCC0.CTRLB & TC0_CCDEN_bm) == 0) {
		return;
	}
	TC0_DisableCCChannels( &TCC0, TC0_CCDEN_bm);
	STUDENT_FIFTH	= PORTJ.IN & PIN7_bm;
}

ISR(TCC1_CCA_vect)	//BB_FORWARD
{
	if((TCC1.CTRLB & TC1_CCAEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCC1, TC1_CCAEN_bm);
	BB_FORWARD	= PORTH.IN & PIN6_bm;
}

ISR(TCC1_CCB_vect)	//BB_REVERSE
{
	if((TCC1.CTRLB & TC1_CCBEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCC1, TC1_CCBEN_bm);
	BB_REVERSE	= PORTH.IN & PIN7_bm;
}

ISR(TCD0_CCA_vect)	//STUDENT_FORWARD
{
	if((TCD0.CTRLB & TC0_CCAEN_bm) == 0) {
		return;
	}
	TC0_DisableCCChannels( &TCD0, TC0_CCAEN_bm);
	STUDENT_FORWARD	= PORTJ.IN & PIN3_bm;
}

ISR(TCD0_CCB_vect)	//STUDENT_REVERSE
{
	if((TCD0.CTRLB & TC0_CCBEN_bm) == 0) {
		return;
	}
	TC0_DisableCCChannels( &TCD0, TC0_CCBEN_bm);
	STUDENT_REVERSE	= PORTJ.IN & PIN4_bm;
}

ISR(TCD0_CCC_vect)	//STUDENT_LEFT
{
	if((TCD0.CTRLB & TC0_CCCEN_bm) == 0) {
		return;
	}
	TC0_DisableCCChannels( &TCD0, TC0_CCCEN_bm);
	STUDENT_LEFT	= PORTJ.IN & PIN5_bm;
}

ISR(TCD0_CCD_vect)	//STUDENT_RIGHT
{
	if((TCD0.CTRLB & TC0_CCDEN_bm) == 0) {
		return;
	}
	TC0_DisableCCChannels( &TCD0, TC0_CCDEN_bm);
	STUDENT_RIGHT	= PORTJ.IN & PIN6_bm;
}

ISR(TCD1_CCA_vect)	//PANEL_BUMPER_OVERRIDE
{
	if((TCD1.CTRLB & TC1_CCAEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCD1, TC1_CCAEN_bm);
	PANEL_BUMPER_OVERRIDE	= PORTK.IN & PIN1_bm;

	if(PANEL_BUMPER_OVERRIDE == 0) {	//platform down
		PORTK.OUTCLR = PIN4_bm;	//turn on indicator LED
	}
	else {
		PORTK.OUTSET = PIN4_bm;	//turn off indicator LED
	}
}

ISR(TCD1_CCB_vect)	//LIMIT_SWITCH
{
	if((TCD1.CTRLB & TC1_CCBEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCD1, TC1_CCBEN_bm);
	LIMIT_SWITCH	= PORTK.IN & PIN7_bm;
	if(LIMIT_SWITCH == 0) {	//platform down
		PORTK.OUTCLR = PIN5_bm;	//turn on indicator LED
	}
	else {
		PORTK.OUTSET = PIN5_bm;	//turn off indicator LED
	}
}

ISR(TCE1_CCA_vect)	//PANEL_LA_UP
{
	if((TCE1.CTRLB & TC1_CCAEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCE1, TC1_CCAEN_bm);
	PANEL_LA_UP	= PORTK.IN & PIN0_bm;
}

ISR(TCE1_CCB_vect)	//ESTOP
{
	uint8_t temp;
	if((TCE1.CTRLB & TC1_CCBEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCE1, TC1_CCBEN_bm);
	temp = PORTK.IN & PIN2_bm;
	if(temp != ESTOP) {
		ESTOP = temp;
		PulsePGDTEstop();
	}
}

ISR(TCF1_CCA_vect)	//PANEL_LA_DOWN
{
	if((TCF1.CTRLB & TC1_CCAEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCF1, TC1_CCAEN_bm);
	PANEL_LA_DOWN	= PORTK.IN & PIN3_bm;
}

void PulsePGDTEstop(void)
{
	PORTK.OUTSET = PIN6_bm;
	//start pulse timer
	TC_SetCompareB(&TCF1, TCF1.CNT + PGDT_ESTOP_PULSE_PERIOD);
	TC1_EnableCCChannels( &TCF1, TC1_CCBEN_bm);

	ResetBumperThreshold();
}

ISR(TCF1_CCB_vect)
{
	if((TCF1.CTRLB & TC1_CCBEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCF1, TC1_CCBEN_bm);
	PORTK.OUTCLR = PIN6_bm;
}
