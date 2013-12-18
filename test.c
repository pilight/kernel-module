#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <poll.h>
#include <unistd.h>
 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
 
#include <string.h>

#define POLL_TIMEOUT	1000

int main(void) {
	int f = -1;
	int i = 0;
	int ret = 0;
	char buff[255] = {0};

	if((f = open("/dev/pilight", O_RDONLY)) == -1) {
		perror("open");
		return;
	}

	// if(ioctl(f, 0, 18) != 0) {
		// printf("cannot claim gpio %d\n", 18);
	// }
	// ioctl(f, 1, 150);
	// ioctl(f, 10, 20000);
	while(1) {
		ret=read(f, buff, 10);
		if(ret < 0) {
			perror("read()");
			return 4;
		}
		printf("%d %s\n", ret, buff);
	}	
	
	close(f);
}