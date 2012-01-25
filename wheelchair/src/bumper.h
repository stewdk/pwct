/*
 * bumper.h
 *
 *  Created on: Mar 28, 2011
 *      Author: grant
 */

#ifndef BUMPER_H_
#define BUMPER_H_

/**< Buffer Size */
#define BUFFER_SIZE  16

typedef uint16_t KeyType;

typedef struct
{
	uint16_t writePointer; /**< write pointer */
	uint16_t readPointer;  /**< read pointer */
	uint16_t size;         /**< size of circular buffer */
	KeyType keys[0];    /**< Element of circular buffer */
} CircularBuffer;

CircularBuffer* BB[7];
CircularBuffer* Temp;

int8_t initBumpers(void);

void BumperAlgorithm(void);

void BufferStore(uint8_t currB, int16_t new_data);

void GetBumperValues(uint16_t *values);

void PrintRightCornerBumper(void);
void PrintLeftCornerBumper(void);
void PrintRightFrontBumper(void);

#endif /* BUMPER_H_ */
