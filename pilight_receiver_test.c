#include <stdlib.h>
#include <stdio.h>
#include <signal.h>
#include <unistd.h>
 
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
 
#include <string.h>
 
#define PULSE_VALID_SIGNAL     40
#define PULSE_INVALID_SIGNAL   60
 
#define DEVICE_PATH "/dev/pilight0"
 
 
/****************************************************************************/
/* Function which will write program pid to device secified by path,        */
/****************************************************************************/
void reloadKernelHandler() {
   int devHandle  = -1;
   char buff[10] = { 0 }; 
 
   if ( (devHandle = open(DEVICE_PATH, O_WRONLY)) == -1) {
      perror("open");
      return;
   }
 
   sprintf(buff, "%d\n", getpid());
 
   if (write(devHandle, buff, strlen(buff)) == -1)
      perror("write");
 
   close(devHandle);
 
   return;
}
 
 
/****************************************************************************/
/* Function triggered by signal                                             */
/****************************************************************************/
static void run_function_valid(int sig, siginfo_t *siginfo, void *c)
{
   int devHandle  = -1;
   char buff[10] = { 0 }; 
   int pulse_length = 0;
      
   if ( (devHandle = open(DEVICE_PATH, O_RDONLY)) == -1) {
      perror("open");
      return;
   }
   
   if (read(devHandle, buff, 10) == -1)
      perror("read");
   
   sscanf(buff, "%d", &pulse_length);
   
   close(devHandle);
   
   printf ("Pulse of length %d received!\n", pulse_length);
 
   //usleep(100);
 
   // reenable interrupt in kernel, and once again write own pid
   reloadKernelHandler();
}

/****************************************************************************/
/* Function triggered by signal                                             */
/****************************************************************************/
static void run_function_invalid(int sig, siginfo_t *siginfo, void *c)
{
   // int devHandle  = -1;
   // char buff[10] = { 0 }; 
   // int pulse_length = 0;
      
   // if ( (devHandle = open(DEVICE_PATH, O_RDONLY)) == -1) {
      // perror("open");
      // return;
   // }
   
   // if (read(devHandle, buff, 10) == -1)
      // perror("read");
   
   // sscanf(buff, "%d", &pulse_length);
   
   // close(devHandle);
   
   // printf ("Pulse of length %d received!\n", pulse_length);
 
   usleep(10000);
 
   // reenable interrupt in kernel, and once again write own pid
   reloadKernelHandler();
}

 
/****************************************************************************/
/* Main.                                                                    */
/****************************************************************************/
int main(int argc, char *argv[]) {
 
   struct sigaction act_iv = { 0 };
   struct sigaction act_v = { 0 };
 
   act_iv.sa_sigaction = &run_function_invalid;
   act_iv.sa_flags = SA_SIGINFO;
 
   act_v.sa_sigaction = &run_function_valid;
   act_v.sa_flags = SA_SIGINFO;
 
   // attach own function to signal 
   
   if (sigaction(PULSE_VALID_SIGNAL, &act_v, NULL) < 0) {
      perror ("error");
      return 1;
   }
   
   if (sigaction(PULSE_INVALID_SIGNAL, &act_iv, NULL) < 0) {
      perror ("error");
      return 1;
   }
 
   // enable kernel interrupt and send own pid to kernel
   reloadKernelHandler();
 
   // loop
   while(1) {
		sleep(1);
}
 
   return 0;
}