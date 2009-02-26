//-----------------------------------------------------------------------------
//
// 8051hardware.h
// Copyright ©2009 Bryan Henry <dev@apogee-dev.com>  
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

#ifndef __INCLUDED_8051hardware__
#define __INCLUDED_8051hardware__

struct PortPin {
	unsigned char port;
	unsigned char pin;
};

typedef enum {
	DigitalInput,
	AnalogInput,
	DigitalOutput
} IOType;

struct PortPin* newPortPin(unsigned char port, unsigned char pin);

void configurePinIO(struct PortPin* p, IOType type);
void configurePinIOWithMask(unsigned char portNum, unsigned char bitMask, IOType type);

// Digital input
bit getPinState(struct PortPin* pin);

// Analog-digital conversion
void configureADCOnPin(struct PortPin *p);
unsigned char performADC();

#endif
