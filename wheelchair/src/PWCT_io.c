/*
 * PWCT_io.c
 *
 *  Created on: Mar 28, 2011
 *      Author: grant
 */

#include <util/delay.h>
#include <stdio.h>
#include "../atmel/avr_compiler.h"
#include "../atmel/TC_driver.h"
#include "../atmel/port_driver.h"
#include "../atmel/pmic_driver.h"
#include "../atmel/adc_driver.h"
#include "nordic_driver.h"
#include "util.h"
#include "PWCT_io.h"

//debounce timers 1tick = 8us
//1250 ticks = 10ms
#define DEBOUNCE_PERIOD 1250

//pulse timer 1tick = 8us
//31250 ticks = 250ms
#define PGDT_ESTOP_PULSE_PERIOD 31250

#define ENABLE_INSTRUCTOR_REMOTE_LINEAR_ACTUATOR_CONTROL	0

/* Input/Output List
 *  Item							Pin			In/Out		Note
 *  Prop. Joy Detect				PK7			Input		Debounce
 *  Prop. Joy Speed (Fwd/Rev)		PA1			Input		A/D
 *  Prop. Joy Direction (L/R)		PA2			Input		A/D
 *	Bumper							PA7			Input		optional A/D
 *	Limit Switch 1					PA3			Input		A/D
 *	Limit Switch 2					PA4			Input		A/D
 *	Limit Switch 3					PA5			Input		A/D
 *	Limit Switch 4					PA6			Input		A/D
 *	Instructor Forward				remote		Input		Clean
 *	Instructor Reverse				remote		Input		Clean
 *	Instructor Left					remote		Input		Clean
 *	Instructor Right				remote		Input		Clean
 *	Instructor LA Up				remote		Input		Clean
 *	Instructor LA Down				remote		Input		Clean
 *	Instructor Disable Student Joy	remote		Input		Clean
 *	Student Forward					PJ3			Input		Debounce
 *	Student Reverse					PJ4			Input		Debounce
 *	Student Left					PJ5			Input		Debounce
 *	Student Right					PJ6			Input		Debounce
 *	Student Fifth					PJ7			Input		Debounce
 *	Buddy Button Forward			PH6			Input		Debounce
 *	Buddy Button Reverse			PH7			Input		Debounce
 *	Buddy Button Left				PJ0			Input		Debounce
 *	Buddy Button Right				PJ1			Input		Debounce
 *	Buddy Button Fifth				PJ2			Input		Debounce
 *	Emergency Stop					PK2			Input		Debounce
 *	Omni+ On/Off Switch				PK6			Output
 *	Panel LA Up						PK0			Input		Debounce
 *	Panel LA Down					PK3			Input		Debounce
 *	Panel LA LED					PK5			Output
 *	Panel Bumper Override LED		PK4			Output
 *	Panel Bumper Override Switch	PK1			Input		Debounce
 *	Omni+ Out Forward				PH1			Output
 *	Omni+ Out Reverse				PH0			Output
 *	Omni+ Out Left					PH4			Output
 *	Omni+ Out Right					PH3			Output
 *	Omni+ Out Fifth					PH5			Output
 *	LCD Button 1					PQ0			Input		Debounce
 *	LCD Button 2					PQ1			Input		Debounce
 *	LCD Button 3					PQ2			Input		Debounce
 *	LCD Button 4					PQ3			Input		Debounce
 *	LCD Button 5					PR0			Input		Debounce
 *	Fwd/Rev Invert					PR1			Input		Debounce
 */

//these input flags are all active low
static uint8_t INSTRUCTOR_FORWARD;
static uint8_t INSTRUCTOR_REVERSE;
static uint8_t INSTRUCTOR_LEFT;
static uint8_t INSTRUCTOR_RIGHT;
static uint8_t INSTRUCTOR_LA_UP;
static uint8_t INSTRUCTOR_LA_DOWN;
static uint8_t INSTRUCTOR_ESTOP;
static uint8_t BB_FORWARD;
static uint8_t BB_REVERSE;
static uint8_t BB_LEFT;
static uint8_t BB_RIGHT;
static uint8_t BB_FIFTH;
static uint8_t STUDENT_FORWARD;
static uint8_t STUDENT_REVERSE;
static uint8_t STUDENT_LEFT;
static uint8_t STUDENT_RIGHT;
static uint8_t STUDENT_FIFTH;
static uint8_t ESTOP;
static uint8_t PANEL_LA_UP;
static uint8_t PANEL_LA_DOWN;
static uint8_t PANEL_BUMPER_OVERRIDE;
static uint8_t PROP_JOY_DETECT;
static uint8_t INVERT_SWITCH;
static uint8_t LIMIT_SWITCH;

static volatile uint16_t gWiredPropJoySpeed;
static volatile uint16_t gWiredPropJoyDirection;

static void joystickADCsetup(void)
{
	// ADC configuration for proportional joystick
	ADC_CalibrationValues_Load(&ADCA);

	ADC_ConvMode_and_Resolution_Config(&ADCA, ADC_ConvMode_Unsigned, ADC_RESOLUTION_12BIT_gc);

	// External reference on PA0/AREFA
	ADC_Reference_Config(&ADCA, ADC_REFSEL_AREFA_gc);
	ADC_Prescaler_Config(&ADCA, ADC_PRESCALER_DIV512_gc);

	// In Unsigned Single-ended mode, the conversion range is from ground to the reference voltage.
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0, ADC_CH_INPUTMODE_SINGLEENDED_gc, ADC_DRIVER_CH_GAIN_NONE);
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH1, ADC_CH_INPUTMODE_SINGLEENDED_gc, ADC_DRIVER_CH_GAIN_NONE);

	ADC_Ch_InputMux_Config(&ADCA.CH0, ADC_CH_MUXPOS_PIN1_gc, 0);
	ADC_Ch_InputMux_Config(&ADCA.CH1, ADC_CH_MUXPOS_PIN2_gc, 0);

	ADC_FreeRunning_Enable(&ADCA);
	ADC_SweepChannels_Config(&ADCA, ADC_SWEEP_01_gc);

	//ADC_Events_Config(&ADCA, ADC_EVSEL_0123_gc, ADC_EVACT_SYNCHSWEEP_gc );
	ADC_Ch_Interrupts_Config(&ADCA.CH0, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);
	ADC_Ch_Interrupts_Config(&ADCA.CH1, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);

	PMIC.CTRL |= PMIC_MEDLVLEN_bm;

	ADC_Enable(&ADCA);
	ADC_Wait_32MHz(&ADCA);
}

void initPWCTio(void)
{
	//turn off timers
	TC1_ConfigClockSource( &TCF1, TC_CLKSEL_OFF_gc );

	// Set the TC period.
	TC_SetPeriod( &TCF1, 0xFFFF );

	//Set timer in normal mode
	TC1_ConfigWGM( &TCF1, TC_WGMODE_NORMAL_gc );

	TC1_SetCCBIntLevel(&TCF1, TC_CCBINTLVL_MED_gc);

	//start clocks
	TC1_ConfigClockSource( &TCF1, TC_CLKSEL_DIV256_gc );

	PORT_ConfigurePins( &PORTH, PIN6_bm | PIN7_bm,                     false, false, PORT_OPC_PULLUP_gc, PORT_ISC_BOTHEDGES_gc ); // BB fwd, BB rev
	PORT_ConfigurePins( &PORTJ, 0xFF,                                  false, false, PORT_OPC_PULLUP_gc, PORT_ISC_BOTHEDGES_gc ); // remaining BB, switch joystick in
	PORT_ConfigurePins( &PORTK, PIN0_bm | PIN1_bm | PIN3_bm | PIN7_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_BOTHEDGES_gc ); // LA up, bumper override, LA down, Prop. Joy Detect
	PORT_ConfigurePins( &PORTK, PIN2_bm,                               false, false, PORT_OPC_PULLUP_gc, PORT_ISC_FALLING_gc ); // e-stop button
	PORT_ConfigurePins( &PORTQ, PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm, false, false, PORT_OPC_PULLUP_gc, PORT_ISC_BOTHEDGES_gc ); // LCD buttons
	PORT_ConfigurePins( &PORTR, PIN0_bm | PIN1_bm,                     false, false, PORT_OPC_PULLUP_gc, PORT_ISC_BOTHEDGES_gc ); // LCD button, invert switch

	PORT_SetPinsAsInput( &PORTH, PIN6_bm | PIN7_bm);
	PORT_SetPinsAsInput( &PORTJ, PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN4_bm | PIN5_bm | PIN6_bm | PIN7_bm );
	PORT_SetPinsAsInput( &PORTK,  PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm | PIN7_bm);
	PORT_SetPinsAsInput( &PORTQ,  PIN0_bm | PIN1_bm | PIN2_bm | PIN3_bm);
	PORT_SetPinsAsInput( &PORTR,  PIN0_bm | PIN1_bm);

	//set outputs
	PORTK.OUTCLR = PIN4_bm | PIN5_bm;	//leds off
	PORTK.OUTCLR = PIN6_bm;				//Omni+ on/off switch disabled
	PORTK.DIRSET = PIN4_bm | PIN5_bm | PIN6_bm;
	PORTH.OUTCLR = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm;	//switch joystick disabled
	PORTH.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm;

	joystickADCsetup();

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

/*
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
*/

void SampleInputs(void)
{
	//uint8_t panelBumperOverrideOld;

	//panelBumperOverrideOld = PANEL_BUMPER_OVERRIDE;

	BB_FORWARD				= PORTH.IN & PIN6_bm;
	BB_REVERSE				= PORTH.IN & PIN7_bm;
	BB_LEFT					= PORTJ.IN & PIN0_bm;
	BB_RIGHT				= PORTJ.IN & PIN1_bm;
	BB_FIFTH				= PORTJ.IN & PIN2_bm;
	STUDENT_FORWARD			= PORTJ.IN & PIN3_bm;
	STUDENT_REVERSE			= PORTJ.IN & PIN4_bm;
	STUDENT_LEFT			= PORTJ.IN & PIN5_bm;
	STUDENT_RIGHT			= PORTJ.IN & PIN6_bm;
	STUDENT_FIFTH			= PORTJ.IN & PIN7_bm;
	ESTOP					= PORTK.IN & PIN2_bm;
	PANEL_LA_UP				= PORTK.IN & PIN0_bm;
	PANEL_LA_DOWN			= PORTK.IN & PIN3_bm;
	PANEL_BUMPER_OVERRIDE	= PORTK.IN & PIN1_bm;
	PROP_JOY_DETECT			= PORTK.IN & PIN7_bm;
	INVERT_SWITCH			= PORTR.IN & PIN1_bm;
	LIMIT_SWITCH			= 0;

	//printf("\rSTUDENT_FORWARD = %d  BB_FORWARD = %d  STUDENT_REVERSE = %d  BB_REVERSE = %d", STUDENT_FORWARD, BB_FORWARD, STUDENT_REVERSE, BB_REVERSE);
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
	 *	Out Left		PH4
	 *	Out Right		PH3
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

uint8_t getPANEL_BUMPER_OVERRIDE(void)
{
	return PANEL_BUMPER_OVERRIDE;
}

uint16_t getWiredPropJoySpeed(void)
{
	uint16_t returnValue;
	AVR_ENTER_CRITICAL_REGION();
	returnValue = gWiredPropJoySpeed;
	AVR_LEAVE_CRITICAL_REGION();
	return returnValue;
}

uint16_t getWiredPropJoyDirection(void)
{
	uint16_t returnValue;
	AVR_ENTER_CRITICAL_REGION();
	returnValue = gWiredPropJoyDirection;
	AVR_LEAVE_CRITICAL_REGION();
	return returnValue;
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

void PulsePGDTEstop(void)
{
	PORTK.OUTSET = PIN6_bm;
	//start pulse timer
	TC_SetCompareB(&TCF1, TCF1.CNT + PGDT_ESTOP_PULSE_PERIOD);
	TC1_EnableCCChannels( &TCF1, TC1_CCBEN_bm);
}

ISR(TCF1_CCB_vect)  //Omni e-stop
{
	if((TCF1.CTRLB & TC1_CCBEN_bm) == 0) {
		return;
	}
	TC1_DisableCCChannels( &TCF1, TC1_CCBEN_bm);
	PORTK.OUTCLR = PIN6_bm;
}

ISR(ADCA_CH0_vect)
{
	AVR_ENTER_CRITICAL_REGION();
	gWiredPropJoySpeed = ADC_ResultCh_GetWord(&ADCA.CH0);
	AVR_LEAVE_CRITICAL_REGION();
}

ISR(ADCA_CH1_vect)
{
	AVR_ENTER_CRITICAL_REGION();
	gWiredPropJoyDirection = ADC_ResultCh_GetWord(&ADCA.CH1);
	AVR_LEAVE_CRITICAL_REGION();
}
