/*
 * linear_actuator.c
 *
 *  Created on: Mar 26, 2011
 *      Author: grant
 */

#include <avr/io.h>
#include "linear_actuator.h"
#include "../atmel/TC_driver.h"
#include "../atmel/adc_driver.h"
#include "util.h"
#include "stdio.h"
#include "../atmel/pmic_driver.h"

//200 = 6.2us
#define TC_PERIOD 1000

//TODO set up current threshold
#define CURRENT_THRESHOLD_MAX	150
#define CURRENT_THRESHOLD_MIN	-150

static int8_t adcb_offset0, adcb_offset1, adcb_offset2, adcb_offset3;
static int16_t adc_result0, adc_result1, adc_result2, adc_result3;
static uint8_t OVERCURRENT_FLAG;

static void setTop(void) {
	PORTE.OUTSET = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm;
}

static void clrTop(void) {
	PORTE.OUTCLR = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm;
}

static void setBottom(void) {
	PORTE.OUTSET = PIN5_bm;
	PORTF.OUTSET = PIN0_bm | PIN1_bm | PIN3_bm;
}

static void clrBottom(void) {
	PORTE.OUTCLR = PIN5_bm;
	PORTF.OUTCLR = PIN0_bm | PIN1_bm | PIN3_bm;
}

void initLinearActuators(void)
{
	//turn off timers
	TC0_ConfigClockSource( &TCE0, TC_CLKSEL_OFF_gc );

	//Enable output
	clrTop();
	clrBottom();
	PORTE.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm | PIN4_bm | PIN5_bm;
	PORTF.DIRSET = PIN0_bm | PIN1_bm | PIN3_bm;

	/* Set the TC period.
	 * 1000 at 32MHz is 32kHz, above human hearing range*/
	TC_SetPeriod( &TCE0, TC_PERIOD );  // Timer/Counter E0

	/* Configure the TC for single slope mode. */
	TC0_ConfigWGM( &TCE0, TC_WGMODE_NORMAL_gc );

	//set overflow interrupt
	TC0_SetOverflowIntLevel( &TCE0, TC_OVFINTLVL_MED_gc);

	// enable interrupt level
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;

	/************************** ADC CONFIG ******************/
	ADC_CalibrationValues_Load(&ADCB);

	/* Set up ADC B to have signed conversion mode and 12 bit resolution. */
	ADC_ConvMode_and_Resolution_Config(&ADCB, ADC_ConvMode_Signed, ADC_RESOLUTION_12BIT_gc);

	// The ADC has different voltage reference options, controlled by the REFSEL bits in the
	// REFCTRL register. Here an external reference is selected
	ADC_Reference_Config(&ADCB, ADC_REFSEL_AREFA_gc);

	// The clock into the ADC decide the maximum sample rate and the conversion time, and
	// this is controlled by the PRESCALER bits in the PRESCALER register. Here, the
	// Peripheral Clock is divided by 512 ( gives 62.5 KSPS with 32Mhz clock )
	ADC_Prescaler_Config(&ADCB, ADC_PRESCALER_DIV512_gc);

	/* Setup channels*/
	ADC_Ch_InputMode_and_Gain_Config(&ADCB.CH0, ADC_CH_INPUTMODE_DIFF_gc, ADC_DRIVER_CH_GAIN_NONE);
	ADC_Ch_InputMode_and_Gain_Config(&ADCB.CH1, ADC_CH_INPUTMODE_DIFF_gc, ADC_DRIVER_CH_GAIN_NONE);
	ADC_Ch_InputMode_and_Gain_Config(&ADCB.CH2, ADC_CH_INPUTMODE_DIFF_gc, ADC_DRIVER_CH_GAIN_NONE);
	ADC_Ch_InputMode_and_Gain_Config(&ADCB.CH3, ADC_CH_INPUTMODE_DIFF_gc, ADC_DRIVER_CH_GAIN_NONE);

	// Setting up the which pins to convert.
	ADC_Ch_InputMux_Config(&ADCB.CH0, ADC_CH_MUXPOS_PIN4_gc, ADC_CH_MUXNEG_PIN0_gc);
	ADC_Ch_InputMux_Config(&ADCB.CH1, ADC_CH_MUXPOS_PIN5_gc, ADC_CH_MUXNEG_PIN1_gc);
	ADC_Ch_InputMux_Config(&ADCB.CH2, ADC_CH_MUXPOS_PIN6_gc, ADC_CH_MUXNEG_PIN2_gc);
	ADC_Ch_InputMux_Config(&ADCB.CH3, ADC_CH_MUXPOS_PIN7_gc, ADC_CH_MUXNEG_PIN3_gc);

	// Setup Interrupt Mode on complete
//	ADC_Ch_Interrupts_Config(&ADCB.CH0, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);
//	ADC_Ch_Interrupts_Config(&ADCB.CH1, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);
//	ADC_Ch_Interrupts_Config(&ADCB.CH2, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);
//	ADC_Ch_Interrupts_Config(&ADCB.CH3, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);

	// Enable PMIC interrupt level
	PMIC.CTRL |= PMIC_MEDLVLEN_bm;

	// Setup sweep of all 4 virtual channels.
	ADC_SweepChannels_Config(&ADCB, ADC_SWEEP_0123_gc);

	// Before the ADC can be used it must be enabled
	ADC_Enable(&ADCB);

	// Wait until the ADC is ready
	ADC_Wait_32MHz(&ADCB);

	/* Get offset value for ADC B.  */
	OVERCURRENT_FLAG = 0;
	adc_result0 = adc_result1 = adc_result2 = adc_result3 = 0;
	adcb_offset0 = adcb_offset1 = adcb_offset2 = adcb_offset3 = 0;
//	adcb_offset0 = ADC_Offset_Get_Signed(&ADCB, &(ADCB.CH0), true);
//	adcb_offset1 = ADC_Offset_Get_Signed(&ADCB, &(ADCB.CH1), true);
//	adcb_offset2 = ADC_Offset_Get_Signed(&ADCB, &(ADCB.CH2), true);
//	adcb_offset3 = ADC_Offset_Get_Signed(&ADCB, &(ADCB.CH3), true);
//	printf("offsets:%d\t%d\t%d\t%d\n\r", adcb_offset0, adcb_offset1, adcb_offset2, adcb_offset3);

	//start single conversion
//	ADC_Ch_Conversion_Start(&ADCB.CH0);

	//enable free running mode
	ADC_FreeRunning_Enable(&ADCB);
}

int8_t RaisePlatform(void)
{
	int8_t err = 0;

	if(OVERCURRENT_FLAG) {
		return -1;
	}

	//turn off bottom fets
	clrBottom();

	//turn on top fets and start timer to recharge bootstrap
//	TCE0.CNT = 0;
	setTop();
	TC0_ConfigClockSource( &TCE0, TC_CLKSEL_DIV1_gc );

	return err;
}

// Recharge Bootstrap Cap
// necessary for the LT1160 to turn on the top mosfet
ISR(TCE0_OVF_vect)
{
	//turn off top fets
	clrTop();

	//turn on bottom fets
	setBottom();

	//wait a little bit for cap to charge
	asm volatile ("nop");	//31.25ns delay
	asm volatile ("nop");	//31.25ns delay

	//turn off bottom fets
	clrBottom();

	//turn on top fets
	setTop();
}

int8_t LowerPlatform(void)
{
	int8_t err = 0;

	if(OVERCURRENT_FLAG) {
		return -1;
	}

	//turn off raise platform pins and timer
	TC0_ConfigClockSource( &TCE0, TC_CLKSEL_OFF_gc );
	clrTop();

	//turn on lower platform pins
	setBottom();

	return err;
}

int8_t StopPlatform(void)
{
	int8_t err = 0;

	TC0_ConfigClockSource( &TCE0, TC_CLKSEL_OFF_gc );

	clrTop();
	clrBottom();

	return err;
}

//Returns 0 for no overcurrent, else bitwise mask for which linear actuator is over currenting
// 0x01, 0x02, 0x04, 0x08
/*
static int8_t isOverCurrent(void)
{
	return OVERCURRENT_FLAG;
}
*/

void PrintLACurrents(void)
{
	printf("1:%5d 2:%5d 3:%5d 4:%5d F:%d\n\r", adc_result0, adc_result1, adc_result2, adc_result3, OVERCURRENT_FLAG);
}

static void checkForOverCurrent(int16_t adc_result, uint8_t LAnum)
{
	if(adc_result > CURRENT_THRESHOLD_MAX || adc_result < CURRENT_THRESHOLD_MIN) {
		StopPlatform();
		OVERCURRENT_FLAG = 1<<LAnum;
	} else {
		OVERCURRENT_FLAG = 0;
	}

}

ISR(ADCB_CH0_vect)
{
//	dbgLEDtgl();
	//int16_t adc_result0;
	adc_result0 = ADC_ResultCh_GetWord_Signed(&ADCB.CH0, adcb_offset0);
	checkForOverCurrent(adc_result0, 0);
}

ISR(ADCB_CH1_vect)
{
//	dbgLEDtgl();
	//int16_t adc_result1;
	adc_result1 = ADC_ResultCh_GetWord_Signed(&ADCB.CH1, adcb_offset1);
	checkForOverCurrent(adc_result1, 1);
}

ISR(ADCB_CH2_vect)
{
//	dbgLEDtgl();
	//int16_t adc_result2;
	adc_result2 = ADC_ResultCh_GetWord_Signed(&ADCB.CH2, adcb_offset2);
	checkForOverCurrent(adc_result2, 2);
}

ISR(ADCB_CH3_vect)
{
//	dbgLEDtgl();
	//int16_t adc_result3;
	adc_result3 = ADC_ResultCh_GetWord_Signed(&ADCB.CH3, adcb_offset3);
	checkForOverCurrent(adc_result3, 3);
}
