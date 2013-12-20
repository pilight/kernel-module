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

//#define BENCHMARK

#include "settings.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/time.h>
#include <unistd.h>
#include <poll.h>

#include "pilight_gpio.h"
#include "gc.h"
#include "log.h"
#include "wiringPi.h"

#include <signal.h>
#include <unistd.h>

#include <string.h>
 

#define DEVICE_PATH "/dev/pilight0"

// max buffer: 6*255
// 6 because max pulse length = 20000 -> 5 + ; = 6
#define MAX_BUFFER_SIZE       1530

#define POLL_TIMEOUT				1000

#define IOCTL_GPIO_IN				10
#define IOCTL_LONGEST_V_P			11
#define IOCTL_SHORTEST_V_P			12
#define IOCTL_START_RECEIVER		13
#define IOCTL_STOP_RECEIVER			14

static int f = -1;


// pinToGpio:
//        Take a Wiring pin (0 through X) and re-map it to the BCM_GPIO pin
//        Cope for 2 different board revisions here.

static int *pinToGpio ;

static int pinToGpioR1 [64] =
{
  17, 18, 21, 22, 23, 24, 25, 4,        // From the Original Wiki - GPIO 0 through 7:        wpi  0 -  7
   0,  1,                                // I2C  - SDA0, SCL0                                wpi  8 -  9
   8,  7,                                // SPI  - CE1, CE0                                wpi 10 - 11
  10,  9, 11,                                 // SPI  - MOSI, MISO, SCLK                        wpi 12 - 14
  14, 15,                                // UART - Tx, Rx                                wpi 15 - 16

// Padding:

      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,        // ... 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,        // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,        // ... 63
} ;

static int pinToGpioR2 [64] =
{
  17, 18, 27, 22, 23, 24, 25, 4,        // From the Original Wiki - GPIO 0 through 7:        wpi  0 -  7
   2,  3,                                // I2C  - SDA0, SCL0                                wpi  8 -  9
   8,  7,                                // SPI  - CE1, CE0                                wpi 10 - 11
  10,  9, 11,                                 // SPI  - MOSI, MISO, SCLK                        wpi 12 - 14
  14, 15,                                // UART - Tx, Rx                                wpi 15 - 16
  28, 29, 30, 31,                        // New GPIOs 8 though 11                        wpi 17 - 20

// Padding:

                      -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,        // ... 31
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,        // ... 47
  -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1, -1,        // ... 63
} ;

int getWiringPiPin(int pin)
{	
	int boardRev;
	
	boardRev = piBoardRev () ;
	if (boardRev == 1){
		pinToGpio =  pinToGpioR1 ;
	}
	else{
		pinToGpio =  pinToGpioR2 ;
	}
	
	return pinToGpio[pin & 63];
}


void initPilightReceiver(int gpio_in, int shortest_pw, int longest_pw, int minimal_cmd_length)
{
	if((f = open(DEVICE_PATH, O_RDONLY)) == -1) {
		perror("open");
		return;
	}
	
	ioctl(f, IOCTL_GPIO_IN, getWiringPiPin(gpio_in));
	ioctl(f, IOCTL_LONGEST_V_P, longest_pw);
	ioctl(f, IOCTL_SHORTEST_V_P, shortest_pw);
	ioctl(f, IOCTL_START_RECEIVER, 0);
	
	return;
}

void deinitPilightReceiver(void)
{
	ioctl(f, IOCTL_STOP_RECEIVER, 0);
		close(f);
	return;
}

int readPilightReceiver(void)
{
	int ret = 0;
	char buff[255] = {0};
	
	ret=read(f, buff, 10);
	if(ret < 0) {
		perror("read()");
		return 0;
	}
	return strtol(buff, NULL, 10);
}

