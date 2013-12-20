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

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "../../pilight.h"
#include "common.h"
#include "log.h"
#include "hardware.h"
#include "wiringPi.h"
#include "gpio.h"
#include "pilight_gpio.h"

#define SHORTEST_PULSEWIDTH 150
#define LONGEST_PULSEWIDTH 20000
#define MINIMAL_CMD_LENGTH 45

int gpio_in = 0;
int gpio_out = 0;

unsigned short gpioHwInit(void) {
	char *mode = NULL;
	int allocated = 0;

	gpio_in = GPIO_IN_PIN;
	gpio_out = GPIO_OUT_PIN;

	if(settings_find_string("hw-mode", &mode) != 0) {
		mode = malloc(strlen(HW_MODE)+1);
		strcpy(mode, HW_MODE);
		allocated = 1;
	}
	settings_find_number("gpio-receiver", &gpio_in);
	settings_find_number("gpio-sender", &gpio_out);

	if(wiringPiSetup() == -1) {
		if(allocated) {
			sfree((void *)&mode);
		}
		return EXIT_FAILURE;
	}
	pinMode(gpio_out, OUTPUT);
	/*if(wiringPiISR(gpio_in, INT_EDGE_BOTH) < 0) {
		logprintf(LOG_ERR, "unable to register interrupt for pin %d", gpio_in) ;
		if(allocated) {
			sfree((void *)&mode);
		}
		return EXIT_SUCCESS;
	}
	if(allocated) {
		sfree((void *)&mode);
	}*/
	initPilightReceiver(gpio_in,SHORTEST_PULSEWIDTH,LONGEST_PULSEWIDTH,MINIMAL_CMD_LENGTH);
	return EXIT_SUCCESS;
}

unsigned short gpioHwDeinit(void) {
	FILE *fd;
	/*if((fd = fopen ("/sys/class/gpio/unexport", "w"))) {
		fprintf(fd, "%d\n", wpiPinToGpio(gpio_in));
		fclose(fd);
	}*/
   deinitPilightReceiver();
	if((fd = fopen ("/sys/class/gpio/unexport", "w"))) {
		fprintf(fd, "%d\n", wpiPinToGpio(gpio_out));
		fclose(fd);
	}
	return 0;
}

int gpioSend(int *code) {
	unsigned short i = 0;

	while(code[i]) {
		digitalWrite(gpio_out, 1);
		usleep((__useconds_t)code[i++]);
		digitalWrite(gpio_out, 0);
		usleep((__useconds_t)code[i++]);
	}
	return 0;
}

int gpioReceive(void) {
	return readPilightReceiver();
}

void gpioInit(void) {

	hardware_register(&gpio);
	hardware_set_id(gpio, "gpio");

	piHiPri(55);

	gpio->init=&gpioHwInit;
	gpio->deinit=&gpioHwDeinit;
	gpio->send=&gpioSend;
	gpio->receive=&gpioReceive;
}