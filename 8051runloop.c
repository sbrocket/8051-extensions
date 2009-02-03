//-----------------------------------------------------------------------------
//
// 8051runloop.c
// Copyright ©2009 Bryan Henry <dev@apogee-dev.com>  
//
// 8051 Runloop - A simple runloop implementation for 8051-powered devices
//
// This is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This software is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
// 
// You should have received a copy of the GNU General Public License
// along with this software.  If not, see <http://www.gnu.org/licenses/>.
//
//-----------------------------------------------------------------------------

#include "8051runloop.h"

#include <c8051_SDCC.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <limits.h>

struct PortPin {
	unsigned char port;
	unsigned char pin;
};

//-----------------------------------------------------------------------------
// Private function prototypes
//-----------------------------------------------------------------------------

void initTimer0();

void growSchedulingArrays();
void growEventRegisterArrays();

bit getPinState(struct PortPin* pin);
void checkForNullPtr(void *p);
float roundNum(float n);

//-----------------------------------------------------------------------------
// Global declaractions
//-----------------------------------------------------------------------------

unsigned int timerReloadVal;

// Globals used for callback scheduling
unsigned char scheduledCount, maxScheduleSize;
unsigned long millisecondCount;

timedCallbackFunc** timedCallbackArray;
unsigned long* timeScheduledArray;

// Globals used for event registering
unsigned char registeredCount, maxRegisterSize;
unsigned long lastPoll;

eventCallbackFunc** eventCallbackArray;
struct PortPin* registeredPins;
unsigned char* curPinStates;

//-----------------------------------------------------------------------------
// Public functions
//-----------------------------------------------------------------------------

void initRunLoop() {
	initTimer0();
	
	timerReloadVal = 0;
	
	millisecondCount = 0;
	scheduledCount = 0;
	maxScheduleSize = 5;
	
	timedCallbackArray = (timedCallbackFunc**)malloc(maxScheduleSize*sizeof(timedCallbackFunc*));
	checkForNullPtr(timedCallbackArray);
	timeScheduledArray = (unsigned long*)malloc(maxScheduleSize*sizeof(unsigned long));
	checkForNullPtr(timeScheduledArray);
	
	registeredCount = 0;
	lastPoll = 0;
	maxRegisterSize = 5;
	
	eventCallbackArray = (eventCallbackFunc**)malloc(maxRegisterSize*sizeof(eventCallbackFunc*));
	checkForNullPtr(eventCallbackArray);
	registeredPins = (struct PortPin*)malloc(maxRegisterSize*sizeof(struct PortPin));
	checkForNullPtr(registeredPins);
	curPinStates = (unsigned char*)malloc((char)ceilf(maxRegisterSize/8.0)*sizeof(unsigned char));
	checkForNullPtr(curPinStates);
}

void runLoopCycle() {
	unsigned int i;
	unsigned char bitMask;
	bit oldPinState, pinState;
	
	// Check whether any registered input pins have changed state
	// Polls pins at a freqency of ~100Hz
	if (millisecondCount - lastPoll > 10) {
		for (i = 0; i < registeredCount; ++i) {
			bitMask = 0x01 << (i % 8);
			oldPinState = curPinStates[i/8] & bitMask;
			pinState = getPinState(&registeredPins[i]);
			
			if (oldPinState != pinState) {
				(*(eventCallbackArray[i]));
				if (pinState)
					curPinStates[i/8] |= bitMask;
				else
					curPinStates[i/8] &= ~bitMask;
				}
		}
	}
	
	// Check whether any scheduled callbacks have expired
	for (i = scheduledCount; i > 0; --i) {
		if (timeScheduledArray[i-1] > millisecondCount)
			break;
		
		(*(timedCallbackArray[i-1]));	// call scheduled function
		--scheduledCount;
	}

	printf("<DEBUG> Millsecond count: %d\n\r", millisecondCount);
}

void waitForTime(float sec) {
	unsigned long endTime;
	
	endTime = millisecondCount + (unsigned char)(sec*1000);
	while (millisecondCount < endTime) {
		// Pausing other execution until the endTime is reached
	}
}

void scheduleTimedCallbackInRunLoop(timedCallbackFunc funcPtr, float sec) {
	unsigned int i, insertInd;
	unsigned long timeToSchedule;
	
	if (scheduledCount == maxScheduleSize)
		growSchedulingArrays();
	
	timeToSchedule = millisecondCount + (unsigned char)(sec*1000);
	insertInd = scheduledCount;
	while (insertInd > 0 && timeScheduledArray[insertInd-1] < timeToSchedule) { 
		--insertInd;
	}
	
	for (i = scheduledCount; i > insertInd; --i) {
		timedCallbackArray[i] = timedCallbackArray[i-1];
		timeScheduledArray[i] = timeScheduledArray[i-1];
	}
	
	timedCallbackArray[insertInd] = funcPtr;
	timeScheduledArray[insertInd] = timeToSchedule;
	
	++scheduledCount;
}


void registerForEventCallbacksOnPinInRunLoop(eventCallbackFunc funcPtr, unsigned char port, unsigned char pin) {
	unsigned char bitMask, ind = registeredCount;

	if (registeredCount == maxRegisterSize)
		growEventRegisterArrays();

	if (port > 3 || pin > 7) {
		printf("<WARNING> Attempted to register for events on invalid pin P%u.%u!\n\r", port, pin);
		return;
	}
	
	eventCallbackArray[ind] = funcPtr;
	registeredPins[ind].port = port;
	registeredPins[ind].pin = pin;

	bitMask = 0x01 << (ind % 8);
	if (getPinState(&registeredPins[ind]))
		curPinStates[ind/8] |= bitMask;
	else
		curPinStates[ind/8] &= ~bitMask;
	
	++registeredCount;
}

//-----------------------------------------------------------------------------
// Private Functions
//-----------------------------------------------------------------------------

void initTimer0() {
	if (!timerReloadVal)
		timerReloadVal = roundNum(65536-MILLISECOND_GRANULARITY/(1/(SYSTEM_CLOCK/12.0))/1000);
	
	CKCON &= ~0x08;				// set Timer0 source to SYSCLK/12
	TMOD &= ~0x0E;				
	TMOD |= 0x01;				// set Timer0 to Mode 1 (16-bit timer), etc
	EA = 1;						// enable global interrupts
	ET0 = 1;					// enable Timer0 interrupt
	
	TH0 = timerReloadVal / 256;
	TL0 = timerReloadVal % 256;	// reset Timer0 counter to calc'd reload value
	TR0 = 1;					// enable Timer0
}

void timer0Interrupt() {
	millisecondCount += MILLISECOND_GRANULARITY;
	
	// Reset the timer
	TR0 = 0;					// disable Timer0
	TH0 = timerReloadVal / 256;
	TL0 = timerReloadVal % 256;	// reset Timer0 counter to calc'd reload value
	TR0 = 1;					// enable Timer0
}

void growSchedulingArrays() {
	// Determine the new size for the arrays
	if (maxScheduleSize <= UCHAR_MAX/2) {
		maxScheduleSize *= 2;
	} else if (maxScheduleSize == UCHAR_MAX) {
		printf("<ERROR> Attempted to schedule more than UCHAR_MAX (%u) timer callbacks!\n\r", UCHAR_MAX);
		printf("<ERROR> Exiting - unable to continue.\n\r");
		*(NULL);		// no exit() func in SDCC stdlib.h, so we make our own
	} else {
		maxScheduleSize = UCHAR_MAX;
	}
	
	// Grow the arrays using realloc
	timedCallbackArray = (timedCallbackFunc**)realloc(timedCallbackArray, maxScheduleSize*sizeof(timedCallbackFunc*));
	checkForNullPtr(timedCallbackArray);
	timeScheduledArray = (unsigned long*)realloc(timeScheduledArray, maxScheduleSize*sizeof(unsigned long));
	checkForNullPtr(timeScheduledArray);
}

void growEventRegisterArrays() {
	// Determine the new size for the arrays
	if (maxRegisterSize <= UCHAR_MAX/2) {
		maxRegisterSize *= 2;
	} else if (maxRegisterSize == UCHAR_MAX) {
		printf("<ERROR> Attempted to register more than UCHAR_MAX (%u) events!\n\r", UCHAR_MAX);
		printf("<ERROR> Exiting - unable to continue.\n\r");
		*(NULL);		// no exit() func in SDCC stdlib.h, so we make our own
	} else {
		maxRegisterSize = UCHAR_MAX;
	}
	
	// Grow the arrays using realloc
	eventCallbackArray = (eventCallbackFunc**)realloc(eventCallbackArray, maxRegisterSize*sizeof(eventCallbackFunc*));
	checkForNullPtr(eventCallbackArray);
	registeredPins = (struct PortPin*)realloc(registeredPins, maxRegisterSize*sizeof(struct PortPin));
	checkForNullPtr(registeredPins);
	curPinStates = (unsigned char*)realloc(curPinStates, (char)ceilf(maxRegisterSize/8.0)*sizeof(unsigned char));
	checkForNullPtr(curPinStates);
}

void checkForNullPtr(void *p) {
	if (p == NULL) {
		printf("<ERROR> Unable to allocate necessary memory!\n\r");
		printf("<ERROR> Exiting - unable to continue.\n\r");
		*(NULL);		// no exit() func in SDCC stdlib.h, so we make our own
	}
}

float roundNum(float n) {
	float intPart = floorf(n);
	if (n-intPart >= 0.5)
		return intPart+1;
	else
		return intPart;
}

// Really nasty function, but necessary since there's no
// easier way to work with the sbits
bit getPinState(struct PortPin* p) {
	switch (p->port) {
		case 0:
			switch (p->pin) {
				case 0: return P0_0;
				case 1: return P0_1;
				case 2: return P0_2;
				case 3: return P0_3;
				case 4: return P0_4;
				case 5: return P0_5;
				case 6: return P0_6;
				case 7: return P0_7;
			}
		case 1:
			switch (p->pin) {
				case 0: return P1_0;
				case 1: return P1_1;
				case 2: return P1_2;
				case 3: return P1_3;
				case 4: return P1_4;
				case 5: return P1_5;
				case 6: return P1_6;
				case 7: return P1_7;
			}
		case 2:
			switch (p->pin) {
				case 0: return P2_0;
				case 1: return P2_1;
				case 2: return P2_2;
				case 3: return P2_3;
				case 4: return P2_4;
				case 5: return P2_5;
				case 6: return P2_6;
				case 7: return P2_7;
			}
		case 3:
			switch (p->pin) {
				case 0: return P3_0;
				case 1: return P3_1;
				case 2: return P3_2;
				case 3: return P3_3;
				case 4: return P3_4;
				case 5: return P3_5;
				case 6: return P3_6;
				case 7: return P3_7;
			}
	}

	return 0;
}
