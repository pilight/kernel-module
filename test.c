#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <poll.h>
#include <unistd.h>
 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
 
#include <string.h>

#define POLL_TIMEOUT				1000

#define IOCTL_GPIO_IN				1
#define IOCTL_LONGEST_V_P			2
#define IOCTL_SHORTEST_V_P			3
#define IOCTL_START_RECEIVER			4
#define IOCTL_STOP_RECEIVER			5

int main(void) {
	int f = -1;
	int i = 0;
	int ret = 0;
	int n = 0;
	char buff[255] = {0};

	if((f = open("/dev/pilight0", O_RDONLY)) == -1) {
		perror("open");
		return;
	}
	
	ioctl(f, IOCTL_GPIO_IN, 23)
	ioctl(f, IOCTL_LONGEST_V_P, 20000);
	ioctl(f, IOCTL_SHORTEST_V_P, 150);
	ioctl(f, IOCTL_START_RECEIVER, 0);
	
	while(1) {
		ret=read(f, buff, 10);
		if(ret < 0) {
			perror("read()");
			return 4;
		}
		printf("pulse %d: %s\n", n, buff);
		n++;
	}	

	ioctl(f, IOCTL_STOP_RECEIVER, 0);
	close(f);
}
