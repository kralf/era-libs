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
#include "libkin.h"
#include "libkin2epos.h"
#include "libtrajectory.h"



//#define path_lenght 21;

#define MCI_MAX_VIA_POINTS  50
#define MCI_MAX_VEL_INTERVALS  500



int main(int argc, char **argv)
{
  double theta_end[6];
  double tool_end[6];


  double enc_rev[]        = {500,500,500,500,500,500};
  double i_gear[]         = {0.02,0.02,0.02,0.02,0.005,0.01};
  double i_arm[]          = { 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};
      

  int output_current_limit[]           = {0,2500,2000,2000,2000,300,300};


  int error=0;
  int i;
  int id;
  double pos_err[6];  

  struct timeval  time1, time2;
  struct timezone timez;		
  double pause;                       

  double tool_path[MCI_MAX_VIA_POINTS][7];
  int number_of_via_points;

  system("clear");

  if(argc == 1) {
    printf("No scriptfile specified!\nUse %s 'filename' .\n", argv[0]);
    return 0;
  }	
  printf("scriptfile: %s \n", argv[1]);


  if( read_scriptfile( argv[1], tool_path, &number_of_via_points) )
    exit(1);




  /* Trajectory "Test1" *-/
  number_of_via_points = 8;
  double sp[] = {9.687978, 30.237083, 4.713540, 11.739130, 10.5, 0}; //starting point
  double tool_path2[MCI_MAX_VIA_POINTS][7] = { {sp[0], sp[1], sp[2], sp[3], sp[4], sp[5] , 5},
					     {sp[0]-5, sp[1]+5, sp[2], 0, 0, sp[5] , 4},
					     {sp[0]-5, sp[1]+5, sp[2], 0, 0, sp[5] , 5},
					     {sp[0]-5, sp[1]+5, sp[2]+10, 0, 0, sp[5] , 5},
					     {sp[0]-5, sp[1]+5, sp[2]+10, 0, 0, sp[5] , 5},
					     {sp[0]-5, sp[1]+5, sp[2], 0, 0, sp[5] , 5},
					     {sp[0]-5, sp[1]+5, sp[2], 0, 0, sp[5] , 2},
					     {sp[0], sp[1], sp[2], sp[3], sp[4], sp[5] , 3}};  


  /-* */

  /* Circle xz *-/
  int number_of_via_points = 17;
  double r=4;
  //double cs[6] = { 5, 40, 10,   999, 0, 0 }; //circle start theta1 moving
    double cs[6] = { 5, 40, 10,   11, 0, 0 };  //circle start theta2 moving                          
  double tool_path[MCI_MAX_VIA_POINTS][6] = { {9.687978, 30.237083, 4.713540, 11.739130, 10.5, 0}, //starting point
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
  double tool_path_time[MCI_MAX_VIA_POINTS-1] = {6, .8, .8,.8,.8,.8,.8,.8,.8,.8,.8,.8,.8,.8, .8, 6 };
  /-* */

  //print path: 
  for(i=0; i<number_of_via_points; i++)
    printf("%f %f %f %f %f %f %f\n", tool_path[i][0], tool_path[i][1], tool_path[i][2], tool_path[i][3], tool_path[i][4]
	   , tool_path[i][5], tool_path[i][6]);
  
  

  for(i=0; i<MCI_MAX_VIA_POINTS; i++)
    auto_beta1(tool_path[i]);


  
  double theta_vel[MCI_MAX_VEL_INTERVALS][6];
  int kin_err[MCI_MAX_VEL_INTERVALS][3];
  double dt = 0.15; //seconds

  int number_vel_intervals;




  error =  mci(tool_path,  number_of_via_points, theta_vel, &number_vel_intervals, dt, kin_err);


  for(i=0; i<number_vel_intervals; i++)
    printf("%f, %f, %f, %f, %f, %f   %i %i %i\n",  theta_vel[i][0], theta_vel[i][1], theta_vel[i][2],
	   theta_vel[i][3], theta_vel[i][4], theta_vel[i][5], kin_err[i][0], kin_err[i][1], kin_err[i][2]);


  if (error !=0)
    return 1;
			    




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
	  printf("theta%i soll: %d\tist: %lf\n", id, 0, 
		 ((myepos_read.number[id-1].actual_position)*360*(i_gear[id-1]*i_arm[id-1]))/(enc_rev[id-1]*4));
	  theta_end[id-1] = (myepos_read.number[id-1].actual_position)
	    *2*M_PI*(i_gear[id-1]*i_arm[id-1])/(enc_rev[id-1]*4);
	  //	  printf("%i %lf %lf %lf\n",  
	  // (myepos_read.number[id-1].actual_position),(i_gear[i-1]),(i_arm[i-1]),(enc_rev[i-1]*4));

	}

      theta_tiks_to_rad(theta_end);
      forward_kinematics( tool_end, theta_end); 
      tool_print(tool_end);
      tool_end[0] -= 9.687978;
      tool_end[1] -= 30.237083;
      tool_end[2] -= 4.713540;
      printf("abs: %lf [cm]\n", sqrt(tool_end[0]*tool_end[0]+tool_end[1]*tool_end[1]+tool_end[2]*tool_end[2] ));
      //{9.687978, 30.237083, 4.713540, 11.739130, 10.5, 0}
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



