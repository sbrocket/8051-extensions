//-----------------------------------------------------------------------------
//
// 8051runloop.h
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

#ifndef __INCLUDED_8051runloop__
#define __INCLUDED_8051runloop__

#define MILLISECOND_GRANULARITY 10		// Milliseconds between interrupts
#define SYSTEM_CLOCK SYSCLK				// The system clock speed in Hz
typedef void (*timedCallbackFunc)(void);
typedef void (*eventCallbackFunc)(void);

//-----------------------------------------------------------------------------
// Public function prototypes
//-----------------------------------------------------------------------------
void startRunLoop();
void initRunLoop();
void runLoopCycle();

void pauseAllTimers();
void restartAllTimers();

void waitForTime(float sec);
void scheduleTimedCallbackInRunLoop(timedCallbackFunc funcPtr, float sec);
void registerForEventsOnDigitalInputPin(eventCallbackFunc funcPtr, unsigned char port, unsigned char pin, bit initCall);
void timer0ISR() __interrupt (1);

#endif

