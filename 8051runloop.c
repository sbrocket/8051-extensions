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
#include <stdio.h>
#include <math.h>

//-----------------------------------------------------------------------------
// Private function prototypes
//-----------------------------------------------------------------------------

void initTimer0();
void timer0ISR() interrupt 1;
void makeSpaceAtFrontOfArrays();
void growSchedulingArrays();
void checkForNullPtr();

//-----------------------------------------------------------------------------
// Global declaractions
//-----------------------------------------------------------------------------

unsigned int scheduledCount;
unsigned int maxScheduleSize;
unsigned int timerReloadVal;
unsigned long millisecondCount;

loopCallbackFunc* callbackArray;
unsigned long* timeScheduledArray;

//-----------------------------------------------------------------------------
// Public functions
//-----------------------------------------------------------------------------

void initRunLoop() {
	initTimer0();
	
	timerReloadVal = 0;
	millisecondCount = 0;
	scheduledCount = 0;
	maxScheduleSize = 10;
	
	callbackArray = (loopCallbackFunc**)malloc(maxScheduleSize*sizeof(loopCallbackFunc*));
	checkForNullPtr(callbackArray);
	timeScheduledArray = (unsigned long*)malloc(maxScheduleSize*sizeof(unsigned long));
	checkForNullPtr(timeScheduledArray);
}

void runLoopCycle() {
	// Check whether any scheduled callbacks have expired
	for (unsigned int i = scheduledCount-1; i >= 0; --i) {
		if (timeScheduledArray[i] > millisecondCount)
			break;
		
		(*(callbackArray[i]));	// call scheduled function
		--scheduledCount;
	}
}

void scheduleTimedCallbackInRunLoop(loopCallbackFunc funcPtr, float sec) {
	if (scheduledCount == maxScheduleSize)
		growSchedulingArrays();
	
	unsigned long timeToSchedule = millisecondCount + (unsigned char)(sec*1000);
	unsigned int insertInd = scheduledCount;
	while (insertInd > 0 && timeScheduledArray[insertInd-1] < timeToSchedule) { 
		--i;
	}
	
	for (unsigned int i = scheduledCount; i > insertInd; --i) {
		callbackArray[i] = callbackArray[i-1];
		timeScheduledArray[i] = timeScheduledArray[i-1];
	}
	
	callbackArray[insertInd] = funcPtr;
	timeScheduledArray[insertInd] = timeToSchedule;
	
	++scheduledCount;
}

//-----------------------------------------------------------------------------
// Private Functions
//-----------------------------------------------------------------------------

void initTimer0() {
	if (!timerReloadVal)
		timerReloadVal = round(pow(2,16)-MILLISECOND_GRANULARITY/(1/(SYSTEM_CLOCK/12.0))/1000);
	
	CKCON &= ~0x08;				// set Timer0 source to SYSCLK/12
	TMOD |= 0x01;				// set Timer0 to Mode 1 (16-bit timer), etc
	EA = 1;						// enable global interrupts
	ET0 = 1;					// enable Timer0 interrupt
	
	TH0 = timerReloadVal / 256;
	TL0 = timerReloadVal % 256;	// reset Timer0 counter to calc'd reload value
	TR0 = 1;					// enable Timer0
}

void timer0ISR() interrupt 1 {
	millisecondCount += MILLISECOND_GRANULARITY;
	
	// Reset the timer
	TR0 = 0;					// disable Timer0
	TH0 = timerReloadVal / 256;
	TL0 = timerReloadVal % 256;	// reset Timer0 counter to calc'd reload value
	TR0 = 1;					// enable Timer0
}

void growSchedulingArrays() {
	// Determine the new size for the arrays
	if (maxScheduleSize <= UINT_MAX/2) {
		maxScheduleSize *= 2;
	} else if (maxScheduleSize == UINT_MAX) {
		printf("<ERROR> Attempted to schedule more than UINT_MAX (%d) timer callbacks!\n\r", UINT_MAX);
		printf("<ERROR> Exiting - unable to continue.\n\r");
		exit(1);
	} else {
		maxScheduleSize = UINT_MAX;
	}
	
	// Grow the arrays using realloc
	callbackArray = (loopCallbackFunc**)realloc(callbackArray, maxScheduleSize*sizeof(loopCallbackFunc*));
	checkForNullPtr(callbackArray);
	timeScheduledArray = (unsigned long*)realloc(maxScheduleSize*sizeof(unsigned long));
	checkForNullPtr(timeScheduledArray);
}

void checkForNullPtr(void *p) {
	if (p == NULL) {
		printf("<ERROR> Unable to allocate necessary memory!\n\r");
		printf("<ERROR> Exiting - unable to continue.\n\r");
		exit(2);
	}
}
