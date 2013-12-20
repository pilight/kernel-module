/*
	Copyright (C) 2013 CurlyMo

	This file is part of pilight.

    pilight is free software: you can redistribute it and/or modify it under the
	terms of the GNU General Public License as published by the Free Software
	Foundation, either version 3 of the License, or (at your option) any later
	version.

    pilight is distributed in the hope that it will be useful, but WITHOUT ANY
	WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
	A PARTICULAR PURPOSE.  See the GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with pilight. If not, see	<http://www.gnu.org/licenses/>
*/

#include "settings.h"

#ifndef _PILIGHT_GPIO_H_
#define _PILIGHT_GPIO_H_

void initPilightReceiver(int gpio_in, int shortest_pw, int longest_pw, int minimal_cmd_length);
void deinitPilightReceiver(void);
int readPilightReceiver(void);
void reloadKernelHandler(void);

#endif