/*	Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    20.5.2007
 */


#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <termios.h> 
#include <fcntl.h>   
#include <linux/serial.h>
#include <linux/tty.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <unistd.h>
#include <errno.h>
#include <math.h>
//#include <time.h>


#include "libserial.h"  
#include "kin.h"
#include "kin2serial.h"


//#define path_lenght 21;

int main(void) 
{
  system("clear");

  int id;
  float vel;
  
		
  const  float i_gear[]=          {0.02,0.02,0.02,0.02,0.005,0.01};
  const  float i_arm[]=           { 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};
       
  


  struct timeval time;
  struct timezone timez;

  if(gettimeofday(&time, &timez) == 0)
     usleep(50000);
  /* test timeofday *-/
  if(gettimeofday(&time, &timez) == 0)
    printf("time micro: %06u\n", (long int)time.tv_usec);
  usleep(50000);
  if(gettimeofday(&time, &timez) == 0)
    printf("time micro: %06u\n", (long int)time.tv_usec);
  usleep(50000);
  if(gettimeofday(&time, &timez) == 0)
    printf("time micro: %06u\n", (long int)time.tv_usec);
  usleep(50000);
  if(gettimeofday(&time, &timez) == 0)
    printf("time micro: %06u\n", (long int)time.tv_usec);
  usleep(50000);
  */

  /* Starting Hardware Connection  */
  // User check 
  while(1)
    {
      char c;
      printf("Kinematic Calculations finished.\n Start Hardware Connection? <type 'y'>\n");
      scanf("%c",&c);
      if(c=='y' ) break;
    }

  canHWInit();



  /* test vellocity mode (-2) on theta 6(id=5) */
  id = 5;
  fault_reset(id);
  shutdown(id);
  enable_operation(id);	
  set_velocity_mode_setting_value(id, 0);
  set_mode_of_operation(id,-2);
  shutdown(id);		
  enable_operation(id);	

  vel = -1 / (i_gear[id]*i_arm[id]);

  set_velocity_mode_setting_value(id, vel);
  usleep(500000);
  usleep(500000);
  usleep(500000);
  usleep(500000);
  usleep(500000);
  usleep(500000);
  set_velocity_mode_setting_value(id, 0);


  shutdown(id);	


  /*
  for(id=1;id<7;id++)
    {
      shutdown(id);	
    }	
  */
  
  canHWEnd();

  /* */  


  
  printf("end\n");
  return 0;
}







