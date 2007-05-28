/*	Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    28.5.2007
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
#include "trajectory.h"


//#define path_lenght 21;

#define MCI_MAX_VIA_POINTS  50
#define MCI_MAX_VEL_INTERVALS  500


int main(void) 
{
  system("clear");

  int error=0;

  int id;

  float pos_err[6];  
		

       

  /* follow circle */
  float r = 1;
  float sp[] = {9.687978, 30.237083, 23.513540, 11.739130, 10.5, 0}; //starting point


  float tool_path[MCI_MAX_VIA_POINTS][6] = { {sp[0], sp[1], sp[2], sp[3], sp[4], sp[5] },
					     {sp[0], sp[1], sp[2], 0, sp[4], sp[5] },
					     {sp[0], sp[1], sp[2], 0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(M_PI/6)),    sp[1], sp[2]+r*sin(M_PI/6),    0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(M_PI/3)),    sp[1], sp[2]+r*sin(M_PI/3),    0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(M_PI/2)),    sp[1], sp[2]+r*sin(M_PI/2),    0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(4*M_PI/6)),  sp[1], sp[2]+r*sin(4*M_PI/6),  0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(5*M_PI/6)),  sp[1], sp[2]+r*sin(5*M_PI/6),  0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(M_PI)),      sp[1], sp[2]+r*sin(M_PI),      0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(7*M_PI/6)),  sp[1], sp[2]+r*sin(7*M_PI/6),  0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(8*M_PI/6)),  sp[1], sp[2]+r*sin(8*M_PI/6),  0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(9*M_PI/6)),  sp[1], sp[2]+r*sin(9*M_PI/6),  0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(10*M_PI/6)), sp[1], sp[2]+r*sin(10*M_PI/6), 0, sp[4], sp[5] },
					     {sp[0]+r*(1-cos(11*M_PI/6)), sp[1], sp[2]+r*sin(11*M_PI/6), 0, sp[4], sp[5] },
					     {sp[0], sp[1], sp[2], 0, sp[4], sp[5] },
					     {sp[0], sp[1], sp[2], sp[3], sp[4], sp[5] } };
			     
			     
			     
  float tool_path_time[MCI_MAX_VIA_POINTS-1] = {1, 0, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 1 };
  
  float theta_vel[MCI_MAX_VEL_INTERVALS][6];

  float dt = 0.1; //seconds

  int number_of_via_points = 16;
  int number_vel_intervals;


  error =  mci(tool_path, tool_path_time, number_of_via_points, theta_vel, &number_vel_intervals, dt);


  int i;

  for(i=0; i<number_vel_intervals; i++)
    printf("vel%i: %f, %f, %f, %f, %f, %f\n", i, theta_vel[i][0], theta_vel[i][1], theta_vel[i][2],
	   theta_vel[i][3], theta_vel[i][4], theta_vel[i][5]);

			    



  struct timeval time, time1, time2;
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

  if(error!=0)
    printf("Error occured\n");
  while(1)
    {
      char c;
      printf("Kinematic Calculations finished.\n Start Hardware Connection? <type 'y'>\n");
      scanf("%c",&c);
      if(c=='y' ) break;
    }
  /* */


  
  kin2s_velocity_mode_init();

  
  error = gettimeofday(&time1, &timez);
  for(i=0; i<number_vel_intervals; i++)
    {
      kin2s_velocity_mode_set( theta_vel[i]);
      error = gettimeofday(&time2, &timez);
      usleep( dt*1000000 - (time2.tv_usec-time1.tv_usec) );
      error = gettimeofday(&time1, &timez);
    }
  kin2s_velocity_mode_set_zero();
  printf("velocity mode finished, starting profile position mode\n\n");
  sleep(2);

  /* finish with positon mode (back to zero) */
  kin2s_position_mode_init();

  for(id=1;id<=6;id++)
    {      
      set_profile_velocity( id, 20);
      set_target_position(  id, 0);
    }
  for(id=1;id<=6;id++)
      activate_position(    id);  

      
  do
    {
      for(id=1;id<=5;id++)
	{
	  get_actual_position(id);
	  get_current_actual_value(id);
	  pos_err[id-1] = myepos_read.number[id-1].actual_position  -  0;
	  printf("theta%i soll: %d\tist: %d\n", id, 0, myepos_read.number[id-1].actual_position);
	}
    }
  while(  kin2s_position_error(pos_err) > 3 || pos_err[4]*pos_err[4]>4  );


  
  for(id=1;id<7;id++)
    {
      shutdown(id);	
    }	
  
  
  canHWEnd();

  /* */  


  
  printf("end\n");
  return 0;
}







  /* test vellocity mode (-2) on theta 6(id=5) *-/

  canHWInit();

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

  */
  




