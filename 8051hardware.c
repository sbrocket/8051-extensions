//-----------------------------------------------------------------------------
// 
// 8051hardware.c
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

#include "8051hardware.h"

#include <c8051_SDCC.h>
#include <stdio.h>

struct PortPin* newPortPin(unsigned char port, unsigned char pin) {
	__xdata struct PortPin result;
	result.port = port;
	result.pin = pin;
	
	return &result;
}

bit getPinState(struct PortPin* p) {
	__xdata unsigned char bitMask = 0x1 << p->pin;
	sfr port;
	
	switch (p->port) {
		case 0: port = P0; break;
		case 1: port = P1; break;
		case 2: port = P2; break;
		default: port = P3; break;
	}
	
	return (port & bitMask) ? 1 : 0;
}

void configurePinIO(struct PortPin* p, IOType type) {
	__xdata unsigned char bitMask = 0x1 << p->pin;

	if (type == AnalogInput && p->port != 1) {
		printf("<ERROR> Can't configure ports other than P1 for analog input.\n\r");
		return;
	}
	
	configurePinIOWithMask(p->port, bitMask, type);
}

void configurePinIOWithMask(unsigned char portNum, unsigned char bitMask, IOType type) {
	sfr mdout, port;

	switch (portNum) {
		case 0: mdout = P0MDOUT; port = P0; break;
		case 1: mdout = P1MDOUT; port = P1; break;
		case 2: mdout = P2MDOUT; port = P2; break;
		default: mdout = P3MDOUT; port = P3; break;
	}

	if (type == AnalogInput)
		P1MDIN &= ~bitMask;		// Set input mode to low for analog input

	if (type != DigitalOutput) {
		mdout &= ~bitMask;		// Set output mode to low for open-drain (input)
		port |= bitMask;		// Set pin to high for input
	} else
		mdout |= bitMask;		// Set output mode to high for push-pull (output)
}
