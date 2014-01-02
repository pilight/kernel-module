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

#define IOCTL_GPIO_IN				10
#define IOCTL_LONGEST_V_P			11
#define IOCTL_SHORTEST_V_P			12
#define IOCTL_START_RECEIVER		13
#define IOCTL_STOP_RECEIVER			14
#define IOCTL_FILTER_ON				15
#define IOCTL_GPIO_OUT				16
#define IOCTL_START_TRANSMITTER		17
#define IOCTL_STOP_TRANSMITTER		18

volatile sig_atomic_t stop;

void inithand(int signum){
	stop = 1;
}

int main(void) {
	int f = -1;
	size_t code_len = 0;
	int ret = 0;
	int n = 0;
	char buff[255] = {0};

	int code[255] = { 300, 300, 300, 1000, 300, 300, 300, 1000 };
	
	while(code[code_len]) {
		code_len++;
	}
	code_len++;
	code_len*=sizeof(int);

	if((f = open("/dev/pilight0", O_WRONLY)) == -1) {
		perror("open");
		return;
	}

	ioctl(f, IOCTL_GPIO_OUT, 22);
	ioctl(f, IOCTL_START_TRANSMITTER, 0);
	
	ret = write(f, code, code_len);
	printf("%d %d\n", ret, code_len);	

	if(ret < 0) {
		perror("read()");
		return 4;
	}

	ioctl(f, IOCTL_STOP_TRANSMITTER, 0);
	
	close(f);

	if((f = open("/dev/pilight0", O_RDONLY)) == -1) {
		perror("open");
		return;
	}
	
	ioctl(f, IOCTL_GPIO_IN, 23);
	ioctl(f, IOCTL_LONGEST_V_P, 40000);
	ioctl(f, IOCTL_SHORTEST_V_P, 50);
	ioctl(f, IOCTL_FILTER_ON, 0);
	ioctl(f, IOCTL_START_RECEIVER, 0);
	
	while(! stop) {
		ret=read(f, buff, 10);
		if(ret < 0) {
			perror("read()");
			return 4;
		}
		printf("%s\n", buff);
		n++;
	}	

	ioctl(f, IOCTL_STOP_RECEIVER, 0);
	close(f);

	return 0;
}