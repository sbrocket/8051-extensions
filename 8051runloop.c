//-----------------------------------------------------------------------------
// 
// 8051runloop.c
// Copyright ©2009 Bryan Henry <dev@apogee-dev.com>  
// 
// Purpose: A simple runloop implementation for 8051-powered devices
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
#include "8051hardware.h"

//-----------------------------------------------------------------------------
// Private function prototypes
//-----------------------------------------------------------------------------

void initTimer0();

void growSchedulingArrays();
void growEventRegisterArrays();

void checkForNullPtr(void *p);
float roundNum(float n);
void crash();

//-----------------------------------------------------------------------------
// Global declaractions
//-----------------------------------------------------------------------------

__xdata unsigned char highReloadVal, lowReloadVal, errReload;

// Globals used for callback scheduling
__xdata unsigned char scheduledCount, maxScheduleSize;
__xdata volatile unsigned char errCountdown;
__xdata volatile unsigned long millisecondCount;

__xdata timedCallbackFunc* timedCallbackArray;
__xdata unsigned long* timeScheduledArray;

// Globals used for event registering
__xdata unsigned char registeredCount, maxRegisterSize;
__xdata unsigned long lastPoll;

__xdata eventCallbackFunc* eventCallbackArray;
__xdata struct PortPin* registeredPins;
__xdata unsigned char* curPinStates;

//-----------------------------------------------------------------------------
// Public functions
//-----------------------------------------------------------------------------

void startRunLoop() {
	initRunLoop();
	while (1) {
		runLoopCycle();
	}
}

void initRunLoop() {
	initTimer0();
	
	millisecondCount = 0;
	scheduledCount = 0;
	maxScheduleSize = 5;
	
	timedCallbackArray = (timedCallbackFunc*)malloc(maxScheduleSize*sizeof(timedCallbackFunc));
	checkForNullPtr(timedCallbackArray);
	timeScheduledArray = (unsigned long*)malloc(maxScheduleSize*sizeof(unsigned long));
	checkForNullPtr(timeScheduledArray);
	
	registeredCount = 0;
	lastPoll = 0;
	maxRegisterSize = 5;
	
	eventCallbackArray = (eventCallbackFunc*)malloc(maxRegisterSize*sizeof(eventCallbackFunc));
	checkForNullPtr(eventCallbackArray);
	registeredPins = (struct PortPin*)malloc(maxRegisterSize*sizeof(struct PortPin));
	checkForNullPtr(registeredPins);
	curPinStates = (unsigned char*)malloc((char)ceilf(maxRegisterSize/8.0)*sizeof(unsigned char));
	checkForNullPtr(curPinStates);
}

void runLoopCycle() {
	__xdata unsigned int i;
	__xdata unsigned char bitMask;
	bit oldPinState, pinState;
	
	// Check whether any registered input pins have changed state
	// Polls pins at a freqency of ~10Hz or on every loop if timer is off
	if (millisecondCount - lastPoll > 100 || TR0 == 0) {
		lastPoll = millisecondCount;
		for (i = 0; i < registeredCount; ++i) {
			bitMask = 0x01 << (i % 8);
			oldPinState = curPinStates[i/8] & bitMask;
			pinState = getPinState(&registeredPins[i]);
			
			if (oldPinState != pinState) {
				(*eventCallbackArray[i])();
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
		
		(*timedCallbackArray[i-1])();	// call scheduled function
		--scheduledCount;
	}
}

void pauseAllTimers() {
	TR0 = 0;
}

void restartAllTimers() {
	TR0 = 1;
}

void waitForTime(float sec) {
	__xdata unsigned long endTime;
	
	endTime = millisecondCount + (unsigned char)(sec*1000);
	while (millisecondCount < endTime) {
		// Pausing other execution until the endTime is reached
	}
}

void scheduleTimedCallbackInRunLoop(timedCallbackFunc funcPtr, float sec) {
	__xdata unsigned int i, insertInd;
	__xdata unsigned long timeToSchedule;
	
	if (scheduledCount == maxScheduleSize)
		growSchedulingArrays();
	
	timeToSchedule = millisecondCount + (unsigned long)(sec*1000);
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


void registerForEventsOnDigitalInputPin(eventCallbackFunc funcPtr, unsigned char port, unsigned char pin, bit initCall) {
	__xdata unsigned char bitMask, ind = registeredCount;
	
	if (registeredCount == maxRegisterSize)
		growEventRegisterArrays();
	
	if (port > 3 || pin > 7) {
		printf("<WARNING> Attempted to register for events on invalid pin P%u.%u!\n\r", port, pin);
		return;
	}
	
	eventCallbackArray[ind] = funcPtr;
	registeredPins[ind].port = port;
	registeredPins[ind].pin = pin;
	
	configurePinIO(&registeredPins[ind], DigitalInput);	

	bitMask = 0x01 << (ind % 8);
	if (getPinState(&registeredPins[ind]))
		curPinStates[ind/8] |= bitMask;
	else
		curPinStates[ind/8] &= ~bitMask;
	
	++registeredCount;
	
	if (initCall)
		(*funcPtr)();
}

//-----------------------------------------------------------------------------
// Private Functions
//-----------------------------------------------------------------------------

void initTimer0() {
	__xdata float tRV_Float, tRV_Int, tRV_Dec;
	__xdata unsigned int timerReloadVal;
	tRV_Float = 65536-MILLISECOND_GRANULARITY/(1/(SYSTEM_CLOCK/12.0f))/1000.0f;
	tRV_Dec = modff(tRV_Float, &tRV_Int);
	timerReloadVal = tRV_Int;
	
	highReloadVal = timerReloadVal / 256;
	lowReloadVal = timerReloadVal % 256;
	errReload = (unsigned char)(1/tRV_Dec);
	
	CKCON &= ~0x08;				// set Timer0 source to SYSCLK/12
	TMOD &= ~0x0E;				
	TMOD |= 0x01;				// set Timer0 to Mode 1 (16-bit timer), etc
	EA = 1;						// enable global interrupts
	ET0 = 1;					// enable Timer0 interrupt
	
	TH0 = highReloadVal;
	TL0 = lowReloadVal;			// reset Timer0 counter to calc'd reload value
	errCountdown = errReload;	// reset error correct countdown var to calc'd reload value
	TR0 = 1;					// enable Timer0
}

void timer0ISR() __interrupt (1) {
	millisecondCount += MILLISECOND_GRANULARITY;
	
	// Handles timing error correction when reload values be adjusted
	// to exactly measure MILLISECOND_GRANULARITY milliseconds per interrupt
	if (errReload != 0) {
		--errCountdown;
		if (errCountdown == 0) {
			millisecondCount += MILLISECOND_GRANULARITY;
			errCountdown = errReload;	// reset error correct countdown var to calc'd reload value
		}
	}
	
	// Reset the timer
	TH0 = highReloadVal;
	TL0 = lowReloadVal;			// reset Timer0 counter to calc'd reload value
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
	timedCallbackArray = (timedCallbackFunc*)realloc(timedCallbackArray, maxScheduleSize*sizeof(timedCallbackFunc));
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
		crash();
	} else {
		maxRegisterSize = UCHAR_MAX;
	}
	
	// Grow the arrays using realloc
	eventCallbackArray = (eventCallbackFunc*)realloc(eventCallbackArray, maxRegisterSize*sizeof(eventCallbackFunc));
	checkForNullPtr(eventCallbackArray);
	registeredPins = (struct PortPin*)realloc(registeredPins, maxRegisterSize*sizeof(struct PortPin));
	checkForNullPtr(registeredPins);
	curPinStates = (unsigned char*)realloc(curPinStates, (char)ceilf(maxRegisterSize/8.0)*sizeof(unsigned char));
	checkForNullPtr(curPinStates);
}

void checkForNullPtr(void *p) __reentrant {
	if (p == NULL) {
		printf("<ERROR> Unable to allocate necessary memory!\n\r");
		printf("<ERROR> Exiting - unable to continue.\n\r");
		crash();
	}
}

float roundNum(float n) __reentrant {
	float intPart = floorf(n);
	if (n-intPart >= 0.5)
		return intPart+1;
	else
		return intPart;
}

void crash() {
	// no exit() func in SDCC stdlib.h, so we make our own
	int a = *((int*)NULL);
	a;
}
