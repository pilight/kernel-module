#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <poll.h>
#include <unistd.h>
 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
 
#include <string.h>

#define IOCTL_GPIO_IN				10
#define IOCTL_LONGEST_V_P			11
#define IOCTL_SHORTEST_V_P			12
#define IOCTL_START_RECEIVER		13
#define IOCTL_STOP_RECEIVER			14

int main(void) {
	int f = -1;
	int i = 0;
	int ret = 0;
	char buff[255] = {0};

	if((f = open("/dev/pilight0", O_RDONLY)) == -1) {
		perror("open");
		return;
	}
	
	// ioctl(f, IOCTL_GPIO_IN, 23)
	// ioctl(f, IOCTL_LONGEST_V_P, 20000);
	// ioctl(f, IOCTL_SHORTEST_V_P, 150);
	// ioctl(f, IOCTL_START_RECEIVER, 0);
	
	while(1) {
		ret=read(f, buff, 10);
		if(ret < 0) {
			perror("read()");
			return 4;
		}
		printf("%d\n", atoi(buff));
	}	

	// ioctl(f, IOCTL_STOP_RECEIVER, 0);
	close(f);
}
