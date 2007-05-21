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



int main(void) 
{
  system("clear");

  float theta[6];
  float target[6];
  float pos_err[6];
  int i=0;
  int id=0;
  int path_lenght = 10;
  float vel_max   = 1; //rad/s


  //  inverse_kinematics( &target, &th);
  //  target_print(&target);

  theta_init_start_tiks( theta );
  theta_print_tiks(theta);
  theta_tiks_to_rad(theta);
  forward_kinematics( target, theta);

  target_print(target);
  //  theta_print_rad(theta);
  inverse_kinematics( target, theta);
  // 
  theta_rad_to_tiks(theta);
  //target_print(target);
  theta_print_tiks(theta);
  


  float target_path[10][6] = { {20.932409, 23.874096, 23.513540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 24.013540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 24.513540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 25.013540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 25.513540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 26.013540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 26.513540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 27.013540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 27.513540, -0.204886, -1.466077, 0},
			       {20.932409, 23.874096, 28.013540, -0.204886, -1.466077, 0}};

  //{37.576370, 21.875977, 40.151661, -0.409773 , -0.733066, 0} 
  //{20.932409, 23.874096, 23.513540, -0.204886, -1.466077, 0}

  float theta_path_pos[10][6];
  float theta_path_vel[9][6];

  for(i=0;i<path_lenght;i++)
    {
      inverse_kinematics( target_path[i], theta_path_pos[i]);
      theta_path_pos[i][5]=0; //Gripper
      theta_rad_to_tiks( theta_path_pos[i] );
      theta_print_tiks( theta_path_pos[i] );
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
  
  

  /* Starting Hardware Connection 

  canHWInit();
  kin2s_position_mode_init();


  
  for(i=0; i< path_lenght-1; i++)
    {

      kin2s_position_mode_set( theta_path_pos[i], theta_path_vel[i]);
      
      
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
      while(  kin2s_position_error(  pos_err  ) > 100);
    }

  
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
  



  for(id=1;id<7;id++)
    {
      shutdown(id);	
    }	
  /* */  
  
  canHWEnd();




  
  printf("end\n");
  return 0;
}






/** gives back vellocity value of smooth polynomial profile */
float vel_polynomial(float vmax, 
		     float T, 
		     float x)
{
  return 0;
}


