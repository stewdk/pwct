/*
 * bumper.c
 *
 *  Created on: Mar 28, 2011
 *      Author: grant
 *
 *      Bumper 1: Left Side
 *      Bumper 2: Right Side
 *      Bumper 3: Left Corner
 *      Bumper 4: Front Left
 *      Bumper 5: Front Right
 *      Bumper 6: Right Corner
 *      Bumper 7: Back
 *
 */

#include "avr_compiler.h"
#include "bumper.h"
#include "adc_driver.h"
#include "PWCT_io.h"
#include "stdio.h"
#include "string.h"
#include "stdlib.h"



static uint8_t adca_offset;
static uint8_t currBumper = 1;
static uint16_t ThresholdMax[7] = { 0 };
static uint16_t ThresholdMin[7] = { 0xFFFF };
//static int16_t numItems = 0;
static uint16_t LAST_RIGHT_CORNER_RECALC_VAL;
static uint16_t LAST_LEFT_CORNER_RECALC_VAL;

uint16_t calculateRightCornerThresholdMin(void);
uint16_t calculateLeftCornerThresholdMin(void);

/**< Init Circular Buffer */
CircularBuffer* CircularBufferInit(CircularBuffer** pQue, int size)
{
        int sz = size*sizeof(KeyType)+sizeof(CircularBuffer);
        *pQue = (CircularBuffer*) malloc(sz);
        if(*pQue)
        {
            //printf(":[%d] (%d)\n", size, sz);
            (*pQue)->size=size;
            (*pQue)->writePointer = 0;
            (*pQue)->readPointer  = 0;
        }
        return *pQue;
}

int8_t initBumpers(void)
{
	CircularBufferInit(&BB[0], BUFFER_SIZE);
	CircularBufferInit(&BB[1], BUFFER_SIZE);
	CircularBufferInit(&BB[2], BUFFER_SIZE);
	CircularBufferInit(&BB[3], BUFFER_SIZE);
	CircularBufferInit(&BB[4], BUFFER_SIZE);
	CircularBufferInit(&BB[5], BUFFER_SIZE);
	CircularBufferInit(&BB[6], BUFFER_SIZE);
	CircularBufferInit(&Temp, BUFFER_SIZE);

	int8_t err = 0;

	/************************** ADC CONFIG ******************/
	ADC_CalibrationValues_Load(&ADCB);

	/* Set up ADC A to have unsigned conversion mode and 12 bit resolution. */
	ADC_ConvMode_and_Resolution_Config(&ADCA, ADC_ConvMode_Unsigned, ADC_RESOLUTION_12BIT_gc);

	// The ADC has different voltage reference options, controlled by the REFSEL bits in the
	// REFCTRL register. Here the internal reference is selected
	ADC_Reference_Config(&ADCA, ADC_REFSEL_AREFA_gc);

	// The clock into the ADC decide the maximum sample rate and the conversion time, and
	// this is controlled by the PRESCALER bits in the PRESCALER register. Here, the
	// Peripheral Clock is divided by 512 ( gives 62.5 KSPS with 32Mhz clock,  8.9KSPS per bumper)
	ADC_Prescaler_Config(&ADCA, ADC_PRESCALER_DIV512_gc);

	/* Setup channels*/
	ADC_Ch_InputMode_and_Gain_Config(&ADCA.CH0, ADC_CH_INPUTMODE_SINGLEENDED_gc, ADC_DRIVER_CH_GAIN_NONE);

	// Setting up the which pins to convert.
	ADC_Ch_InputMux_Config(&ADCA.CH0, currBumper<<3, 0);

	// Setup Interrupt Mode on complete
	ADC_Ch_Interrupts_Config(&ADCA.CH0, ADC_CH_INTMODE_COMPLETE_gc, ADC_CH_INTLVL_MED_gc);

	/* Enable PMIC interrupt level low. */
	PMIC.CTRL |= PMIC_MEDLVLEX_bm;

	// Before the ADC can be used it must be enabled
	ADC_Enable(&ADCA);

	// Wait until the ADC is ready
	ADC_Wait_32MHz(&ADCA);

	/* Get offset value for ADC A.  */
	adca_offset = ADC_Offset_Get_Unsigned(&ADCA, &(ADCB.CH0), true);

	//start single conversion
	ADC_Ch_Conversion_Start(&ADCA.CH0);

	return err;
}

int comparator ( const void * elem1, const void * elem2 )
{
	if((uint16_t)elem1 < (uint16_t)elem2) {
		return -1;
	}
	else if((uint16_t)elem1 == (uint16_t)elem2) {
		return 0;
	}
	else {	//elem1 > elem2
		return 1;
	}
}
/*
uint16_t median(uint16_t *buf)
{
	uint16_t tempBuf[sizeof(buf[0])*BUFFER_SIZE];

	memcpy(tempBuf, buf, sizeof(buf[0])*BUFFER_SIZE);

	qsort(tempBuf, BUFFER_SIZE, sizeof(buf[0]), &comparator);

	return tempBuf[BUFFER_SIZE/2];
}
*/

uint16_t avg(uint16_t *buf)
{
	uint8_t i;
	uint32_t sum = 0;

	for(i=0; i < BUFFER_SIZE; i++) {
		sum += buf[i];
	}
	return sum>>4;
}

/*
int max (int16_t x[])
{
	int siz = sizeof x / sizeof x[0];
	int m = x[0];
	for (int i = 1; i < siz; ++i)
	{
		if(x[i] > m)
		{
			m = x[i];
		}
	}
	return m;
}
*/
uint16_t min (uint16_t x[])
{
	int siz = BUFFER_SIZE;
	int m = x[0];
	for (int i = 1; i < siz; ++i)
	{
		if(x[i] < m)
		{
			m = x[i];
		}
	}
	return m;
}

int CircularBufferIsFull(CircularBuffer* que)
{
     return ((que->writePointer + 1) % que->size == que->readPointer);
}

void BumperAlgorithm(void)
{
//	int i,j,k,l,m,n,o,p,q,r,s,t,u,v,w,x,y;
	int var = 0;
	uint16_t currentAvg;

	//if buffers are all full and threshold has never been initialized (first time run, only run once)
	if ((ThresholdMin[0] == 0) &&
			CircularBufferIsFull(BB[0]) &&
			CircularBufferIsFull(BB[1]) &&
			CircularBufferIsFull(BB[2]) &&
			CircularBufferIsFull(BB[3]) &&
			CircularBufferIsFull(BB[4]) &&
			CircularBufferIsFull(BB[5]) &&
			CircularBufferIsFull(BB[6]))
	{

		//Set up all thresholds
		//Left side
		var = 0;
		ThresholdMin[var] = 0.85*avg(BB[var]->keys);
		if(ThresholdMin[var] < 400) {
			ThresholdMin[var] = 400;
		}

		//Right side
		var = 1;
		ThresholdMin[var] = 0.85*avg(BB[var]->keys);
		if(ThresholdMin[var] < 400) {
			ThresholdMin[var] = 400;
		}

		//Left Corner
		var = 2;
		ThresholdMin[var] = calculateLeftCornerThresholdMin();

		//Left Front
		var = 3;
		ThresholdMin[var] = 0.5*avg(BB[var]->keys);
		if(ThresholdMin[var] < 400) {
			ThresholdMin[var] = 400;
		}

		//Right Front
		var = 4;
		ThresholdMin[var] = 0.5*avg(BB[var]->keys);
		if(ThresholdMin[var] < 400) {
			ThresholdMin[var] = 400;
		}

		//Right Corner
		var = 5;
		ThresholdMin[var] = calculateRightCornerThresholdMin();


		//Back
		var = 6;
		ThresholdMin[var] = 0.85*avg(BB[var]->keys);
		if(ThresholdMin[var] < 400) {
			ThresholdMin[var] = 400;
		}
	}

	// thresholds have been initialized, do this every time
	if (ThresholdMin[0] != 0)
	{
		BumperForwardReleased();
		BumperReverseReleased();
		BumperLeftReleased();
		BumperRightReleased();

		//check Left Side Bumper
		var = 0;
//		printf("%d ? %d\n", avg(BB[var]->keys), ThresholdMin[var]);
		if(avg(BB[var]->keys) < ThresholdMin[var]) {
			BumperLeftPressed();
//			printf("Left\n");
		}
		//check Right Side Bumper
		var = 1;
//		printf("%d ? %d\n", avg(BB[var]->keys), ThresholdMin[var]);
		if(avg(BB[var]->keys) < ThresholdMin[var]) {
			BumperRightPressed();
//			printf("Right\n");
		}
		//check Left Corner Bumper
		var = 2;
		currentAvg = avg(BB[var]->keys);
		if(currentAvg < ThresholdMin[var]) {
//			BumperLeftPressed();
//			BumperForwardPressed();
		}
		if(GetMoveDirection() & (0x04 | 0x01)) {	//if the direction being moved is right or reverse, we are off the wall
			ThresholdMin[var] = calculateLeftCornerThresholdMin();
		}
		//recalc right corner threshold if left corner is greater than it was before
		if(LAST_LEFT_CORNER_RECALC_VAL+5 < currentAvg) {
			ThresholdMin[var] = calculateLeftCornerThresholdMin();
		}

		//check Forward Bumper
		var = 3;	//front left
		if(avg(BB[var]->keys) < ThresholdMin[var]) {
//			BumperForwardPressed();
		}
		var = 4;	//front right
		if(avg(BB[var]->keys) < ThresholdMin[var]) {
//			BumperForwardPressed();
		}
		var = 5;	//Right Corner
		currentAvg = avg(BB[var]->keys);
		if(currentAvg < ThresholdMin[var]) {
//			BumperRightPressed();
//			BumperForwardPressed();
		}
		if(GetMoveDirection() & (0x04 | 0x02)) {	//if the direction being moved is left or reverse, we are off the wall
			//LAST_RIGHT_CORNER_RECALC_VAL = currentAvg;
			ThresholdMin[var] = calculateRightCornerThresholdMin();
		}
		//recalc right corner threshold if right corner is greater than it was before
		//TODO check noise on bumpers to get value for constant
		if(LAST_RIGHT_CORNER_RECALC_VAL+5 < currentAvg) {
			ThresholdMin[var] = calculateRightCornerThresholdMin();
		}

		//check Reverse Bumper
		var = 6;
//		printf("%d ? %d\n", avg(BB[var]->keys), ThresholdMin[var]);
		if(avg(BB[var]->keys) < ThresholdMin[var]) {
			BumperReversePressed();
			BumperLeftPressed();
			BumperRightPressed();
//			printf("Back\n");
		}
	}
}

void PrintRightCornerBumper(void)
{
	printf("Thres:%4u\tVal:%4u\trecalcVal:%4u\r\n", ThresholdMin[5], avg(BB[5]->keys), LAST_RIGHT_CORNER_RECALC_VAL);
}

void PrintLeftCornerBumper(void)
{
	printf("Thres:%4u\tVal:%4u\trecalcVal:%4u\r\n", ThresholdMin[2], avg(BB[2]->keys), LAST_LEFT_CORNER_RECALC_VAL);
}

void PrintRightFrontBumper(void)
{
	printf("Thres:%4u\tVal:%4u\r\n", ThresholdMin[4], avg(BB[4]->keys));
}

uint16_t calculateRightCornerThresholdMin(void)
{
	uint8_t var = 5;
	uint16_t threshold, minVal;

	minVal = avg(BB[var]->keys);
	LAST_RIGHT_CORNER_RECALC_VAL = minVal;
	if(minVal > 330) {
		threshold =  300;
	} else {
		threshold = (uint16_t)(minVal*0.8);
		if(threshold > 300) {
			threshold = 300;
		}
	}
	return threshold;
}

uint16_t calculateLeftCornerThresholdMin(void)
{
	uint8_t var = 2;
	uint16_t threshold, minVal;

	minVal = avg(BB[var]->keys);
	LAST_LEFT_CORNER_RECALC_VAL = minVal;
	if(minVal > 330) {
		threshold =  300;
	} else {
		threshold = (uint16_t)(minVal*0.8);
		if(threshold > 300) {
			threshold = 300;
		}
	}
	return threshold;
}

void ResetBumperThreshold(void)
{
	//give bumpers time to refill
	_delay_ms(100);
	memset(ThresholdMax, 0, sizeof(ThresholdMax));
	memset(ThresholdMin, 0, sizeof(ThresholdMin));
}

int CircularBufferIsEmpty(CircularBuffer* que)
{
     return (que->readPointer == que->writePointer);
}

int CircularBufferEnque(CircularBuffer* que, KeyType k)
{
	//int isFull = CircularBufferIsFull(que);
	que->keys[que->writePointer] = k;
	que->writePointer++;
	que->writePointer %= que->size;
	return 1;
}

int CircularBufferPeek(CircularBuffer* que, KeyType *k)
{
	uint16_t tempWPointer = 0;
	tempWPointer = que->writePointer;
	if(tempWPointer == 0 ) {
		tempWPointer = que->size - 1;
	}
	else {
		tempWPointer--;
	}

	//int isFull = CircularBufferIsFull(que);
	*k = que->keys[tempWPointer];

//	*k = que->keys[0];

	return 1;
}

int CircularBufferDeque(CircularBuffer* que)
{
	//int isEmpty =  CircularBufferIsEmpty(que);
	//*pK = que->keys[que->readPointer];
	que->readPointer++;
	que->readPointer %= que->size;
	return 1;
}

void BufferStore(uint8_t currB, int16_t new_data)
{
	KeyType a = new_data, test;

	int index = currB-1;

	if (CircularBufferIsFull(BB[index]))
	{
		CircularBufferDeque((BB[index]));
		CircularBufferEnque(BB[index], a);
		CircularBufferPeek(BB[index], &test);
//		printf("adc data:%d, buf:%d, Current BB Index:%d, Current Bumper:%d\n", a, test, index, currBumper);
	}
	else
	{
		CircularBufferEnque(BB[index], a);
		CircularBufferPeek(BB[index], &test);
//		printf("adc data:%d, buf:%d, Current BB Index:%d, Current Bumper:%d\n", a, test, index, currBumper);
	}
//	_delay_ms(20);
}

void GetBumperValues(uint16_t *values)
{
	CircularBufferPeek(BB[0], &values[0]);
	CircularBufferPeek(BB[1], &values[1]);
	CircularBufferPeek(BB[2], &values[2]);
	CircularBufferPeek(BB[3], &values[3]);
	CircularBufferPeek(BB[4], &values[4]);
	CircularBufferPeek(BB[5], &values[5]);
	CircularBufferPeek(BB[6], &values[6]);
}

ISR(ADCA_CH0_vect)
{
	int16_t adc_result;
	static uint8_t cnt = 0;

	adc_result = ADC_ResultCh_GetWord_Unsigned(&ADCA.CH0, adca_offset);

	if(cnt >= 180) {
		cnt = 0;
		BufferStore(currBumper, adc_result);
		//change ADC pin
		if(currBumper == 7) {
			currBumper = 1;
		}
		else {
			currBumper++;
		}
		ADC_Ch_InputMux_Config(&ADCA.CH0, currBumper<<3, 0);
	} else {
		cnt++;
	}

	//start single conversion
	ADC_Ch_Conversion_Start(&ADCA.CH0);
}
