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


#include "libserial.h"  
#include "kin.h"
#include "kin2serial.h"


//#define path_lenght 21;

int main(void) 
{
  system("clear");


  int error;

  double theta[6];
  double tool[6];
  double pos_err[6];
  int i=0;
  int id=0;

  double vel_max   = 1; //rad/s


  //  inverse_kinematics( &tool, &th);
  //  tool_print(&tool);

  theta_init_start_tiks( theta );
  theta_print_tiks( theta );
  theta_tiks_to_rad( theta );
  theta_print_rad( theta );
  forward_kinematics( tool, theta);

  tool_print(tool);
  //  theta_print_rad(theta);
  error = inverse_kinematics( tool, theta);
  // 
  theta_print_rad(theta);
  theta_rad_to_tiks(theta);

  //tool_print(tool);
  theta_print_tiks(theta);
  

  const int path_lenght = 5;
  /* 
  double tool_path[40][6] = { {9.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0},
			       {9.687978, 27.237083, 23.513540-18.8, 11.739130, 10.5, 0},
			       {9.687978, 27.237083, 23.513540-18.8, -4, 10.5, 0},
			       {9.687978, 27.237083, 23.513540-18.8, 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0} };

  */
  /*      
  double tool_path[40][6] = { {9.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0},
			       {-9.687978, 30.237083, 43.513540-18.8, 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0} };
  */
  
  
  double tool_path[40][6] = { {9.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0},
			       {0.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0},
			       {0.687978, 30.237083, 28.513540-18.8, 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 28.513540-18.8, 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0} };
  

  //			       {0, 19.0, 19.0, 0, 0, 0}, // down zero position
			       /*
  double tool_path[40][6] = { {9.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 24.0  -18.8   , 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 24.5   -18.8  , 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 25.0  -18.8   , 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 25.5  -18.8   , 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 26.0  -18.8   , 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 26.5  -18.8   , 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 27.0  -18.8   , 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 27.5  -18.8   , 11.739130, 10.5, 0},
			       {9.687978, 30.237083, 28.0  -18.8   , 11.739130, 10.5, 0}, //10
			       {10.0    , 30.237083, 28.0  -18.8   , 11.739130, 10.5, 0},
			       {11.0     , 30.237083, 28.0 -18.8    , 11.739130, 10.5, 0},
			       {12.0     , 30.237083, 28.0 -18.8    , 11.739130, 10.5, 0},
			       {13.0     , 30.237083, 28.0 -18.8    , 11.739130, 10.5, 0},
			       {14.0     , 30.237083, 28.0 -18.8    , 11.739130, 10.5, 0}, //15
			       {14.0     , 30.237083, 28.0 -18.8    , 10.0     , 10.5, 0}, 
			       {14.0     , 30.237083, 28.0 -18.8    , 10.0     , 10.5, 0}, 
			       {14.0     , 30.237083, 28.0 -18.8    , 10.0     , 10.5, 0}, 
			       {14.0     , 30.237083, 28.0  -18.8   , 10.0     , 10.5, 0}, 
			       {14.0     , 30.237083, 28.0 -18.8    , 10.0     , 10.5, 0}, 
			       {9.687978, 30.237083, 23.513540-18.8, 11.739130, 10.5, 0} }; //21

			       */
  //{37.576370, 21.875977, 40.151661-18.8, -0.409773 , -0.733066, 0} 
  //{20.932409, 23.874096, 23.513540-18.8, -0.204886, -1.466077, 0}

	

  double theta_path_pos[40][6];
  double theta_path_vel[40-1][6];

  for(i=0;i<path_lenght;i++)
    {
      error = inverse_kinematics( tool_path[i], theta_path_pos[i]);
      theta_path_pos[i][5]=0; //Gripper
      theta_print_rad( theta_path_pos[i] );
      printf("point %i\n",i);
      theta_rad_to_tiks( theta_path_pos[i] );
      //       theta_print_tiks( theta_path_pos[i] );
    }

  for(i=0;i<path_lenght-1;i++)
    {
      kin2s_position_mode_calc_vel( theta_path_pos[i+1], theta_path_pos[i], theta_path_vel[i],  vel_max);
          printf("vel%i: %f %f %f %f %f %f \n", i, theta_path_vel[i][0], theta_path_vel[i][1], 
         theta_path_vel[i][2], theta_path_vel[i][3], theta_path_vel[i][4], theta_path_vel[i][5]);
    }

  
			
		       
  
  // User check 
  while(1)
    {
      char c;
      printf("Kinematic Calculations finished.\n Start Hardware Connection? <type 'y'>\n");
      scanf("%c",&c);
      if(c=='y' ) break;
    }
  
  

  /* Starting Hardware Connection  */

  canHWInit();
  kin2s_position_mode_init();


  
  for(i=1; i< path_lenght; i++)
    {

      kin2s_position_mode_set( theta_path_pos[i], theta_path_vel[i-1]);
      
      
      do
	{
	  for(id=1;id<=5;id++)
	    {
	      get_actual_position(id);
	      get_current_actual_value(id);
	      pos_err[id-1] = myepos_read.number[id-1].actual_position  -  theta_path_pos[i][id-1];
	      printf("theta%i soll: %d\tist: %d\n", id, (int) theta_path_pos[i][id-1], myepos_read.number[id-1].actual_position);
	    }
	}
      while(  kin2s_position_error(pos_err) > 3 || pos_err[4]*pos_err[4]>4  );
      sleep(3);
    }

  /*
  do
    {
      for(id=1;id<=5;id++)
	{
	  get_actual_position(id);
	  get_current_actual_value(id);
	  pos_err[id-1] = myepos_read.number[id-1].actual_position  -  theta_path_pos[i][id-1];
	  printf("last theta%i soll: %d\tist: %d\n", id, (int) theta_path_pos[i][id-1], myepos_read.number[id-1].actual_position);
	}
    }
  while(  kin2s_position_error(  pos_err  ) > 5);
  */



  for(id=1;id<7;id++)
    {
      shutdown(id);	
    }	
  /* */  
  
  canHWEnd();




  
  printf("end\n");
  return 0;
}







