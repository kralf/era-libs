/*	Implementing velocity mode
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
  int output_current_limit[]           = {0,2500,2000,2000,2000,300,300};
  system("clear");

  int error=0;
  int i;
  int id;
  float pos_err[6];  

  struct timeval  time1, time2;
  struct timezone timez;		
  float pause;                       

 

  float sp[] = {9.687978, 30.237083, 4.713540, 11.739130, 10.5, 0}; //starting point


  /* Trajectory "Test1" */
  int number_of_via_points = 8;
  float tool_path[MCI_MAX_VIA_POINTS][6] = { {sp[0], sp[1], sp[2], sp[3], sp[4], sp[5] },
					     {sp[0]-5, sp[1]+5, sp[2], 0, 0, sp[5] },
					     {sp[0]-5, sp[1]+5, sp[2], 0, 0, sp[5] },
					     {sp[0]-5, sp[1]+5, sp[2]+10, 0, 0, sp[5] },
					     {sp[0]-5, sp[1]+5, sp[2]+10, 0, 0, sp[5] },
					     {sp[0]-5, sp[1]+5, sp[2], 0, 0, sp[5] },
					     {sp[0]-5, sp[1]+5, sp[2], 0, 0, sp[5] },
					     {sp[0], sp[1], sp[2], sp[3], sp[4], sp[5] }};  
  float tool_path_time[] = {5, 4, 5, 5, 5, 2, 3 };

  /* */

  /* Circle xz *-/
  int number_of_via_points = 17;
  float r=10;
  //float cs[6] = { 5, 45, 10,   999, 0, 0 }; //circle start theta1 moving
    float cs[6] = { 5, 45, 10,   11, 0, 0 };  //circle start theta2 moving                          
  float tool_path[MCI_MAX_VIA_POINTS][6] = { {9.687978, 30.237083, 4.713540, 11.739130, 10.5, 0}, //starting point
					     { cs[0], cs[1], cs[2], cs[3], cs[4], cs[5] },
					     { cs[0], cs[1], cs[2], cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(1*M_PI/6), cs[1], cs[2]+r*(1-cos(1*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(2*M_PI/6), cs[1], cs[2]+r*(1-cos(2*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(3*M_PI/6), cs[1], cs[2]+r*(1-cos(3*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(4*M_PI/6), cs[1], cs[2]+r*(1-cos(4*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(5*M_PI/6), cs[1], cs[2]+r*(1-cos(5*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(6*M_PI/6), cs[1], cs[2]+r*(1-cos(6*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(7*M_PI/6), cs[1], cs[2]+r*(1-cos(7*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(8*M_PI/6), cs[1], cs[2]+r*(1-cos(8*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(9*M_PI/6), cs[1], cs[2]+r*(1-cos(9*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(10*M_PI/6), cs[1], cs[2]+r*(1-cos(10*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0]+r*sin(11*M_PI/6), cs[1], cs[2]+r*(1-cos(11*M_PI/6)), cs[3], cs[4], cs[5] },
					     { cs[0], cs[1], cs[2], cs[3], cs[4], cs[5] },
					     { cs[0], cs[1], cs[2], cs[3], cs[4], cs[5] },
					     {9.687978, 30.237083, 4.713540, 11.739130, 10.5, 0} };
  float tool_path_time[MCI_MAX_VIA_POINTS-1] = {6, 2, 3,3,3,3,3,3,3,3,3,3,3,3, 2, 6 };
  /-* */



  for(i=0; i<MCI_MAX_VIA_POINTS; i++)
    auto_beta1(tool_path[i]);


  
  float theta_vel[MCI_MAX_VEL_INTERVALS][6];
  float dt = 0.15; //seconds

  int number_vel_intervals;


  error =  mci(tool_path, tool_path_time, number_of_via_points, theta_vel, &number_vel_intervals, dt);
  if (error !=0)
    return 1;
  for(i=0; i<number_vel_intervals; i++)
    printf("%f, %f, %f, %f, %f, %f\n",  theta_vel[i][0], theta_vel[i][1], theta_vel[i][2],
	   theta_vel[i][3], theta_vel[i][4], theta_vel[i][5]);

			    




  /* Starting Hardware Connection  */

  // User check 
  if(error!=0)
    printf("Error occured!!\n");
  while(1)
    {
      char c;
      printf("Kinematic Calculations finished.\n Start Hardware Connection? <type 'y'>\n");
      scanf("%c",&c);
      if(c=='y' ) break;
    }
  /* */

  canHWInit();


  for(id=1;id<7;id++)
    {
      set_output_current_limit(id, output_current_limit[id]);
    }

  kin2s_velocity_mode_init();
  
  error = gettimeofday(&time1, &timez);
  for(i=0; i<number_vel_intervals; i++)
    {
      kin2s_velocity_mode_set( theta_vel[i]);
      //kin2s_velocity_mode_set_zero();
      //      printf("new velocity sent\n\n");
      error = gettimeofday(&time2, &timez);
      pause = ( dt*1000000-((1000000*time2.tv_sec+time2.tv_usec)-
			    (1000000*time1.tv_sec + time1.tv_usec)));
      printf("sleep %f\n", pause );
      if( 0 < pause && pause < dt*1000000 )
	usleep( pause );
      else
	{
	  printf("\t\t\t\t\ttime2.tv_sec %i, time2.tv_usec %i, time1.tv_sec %i, time1.tv_usec %i\n\n\n\n",
		 (int)time2.tv_sec , (int)time2.tv_usec , (int)time1.tv_sec , (int)time1.tv_usec);
	}
      
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



