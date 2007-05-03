/*	Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    3.5.2007
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

  int pos[7];
  int id;

  t_theta th;
  t_target target;
  t_target target_add;

  system("clear");
  
  canHWInit();
  kin2s_init( &th, &target );


  for(id=1;id<7;id++)
    {
      fault_reset(id);
      shutdown(id);
      enable_operation(id);	
      set_mode_of_operation(id,1);
      shutdown(id);		
      enable_operation(id);	
    }


  while(1)
    {
      printf("Add to Position X [cm]: ");
      if(scanf("%f",&(target_add.x)) == 0) break;
      printf("Add to Position Y [cm]: ");
      if(scanf("%f",&(target_add.y)) == 0) break;
      printf("Add to Position Z [cm]: ");
      if(scanf("%f",&(target_add.z)) == 0) break;
      printf("Add to Angle beta1 [deg]: ");
      if(scanf("%f",&(target_add.beta1)) == 0) break;
      printf("Add to Angle beta2 [deg]: ");
      if(scanf("%f",&(target_add.beta2)) == 0) break;

      target.x += target_add.x;
      target.y += target_add.y;
      target.z += target_add.z;
      target.beta1 += target_add.beta1/180*M_PI; 
      target.beta2 += target_add.beta2/180*M_PI; 

      target_print(&target);

      inverse_kinematics( &target, &th);
      theta_print_rad( &th );
      theta_rad_to_tiks( &th );
      theta_print_tiks( &th );



      pos[1] = (int)th.theta1;
      pos[2] = (int)th.theta2;
      pos[3] = (int)th.theta3;
      pos[4] = (int)th.theta4;
      pos[5] = (int)th.theta6;

      for(id=1;id<6;id++)
	{
	  set_profile_velocity(id,1);
	  set_target_position(id,pos[id]);
	  activate_position(id);
	  //get_statusword(id);
	}	
		
      do
	{
	  for(id=1;id<6;id++)
	    {
	      get_actual_position(id);
	      get_current_actual_value(id);
	  
	      printf("theta%i soll: %d\tist: %d\n", id, (int) pos[id], myepos_read.number[id-1].actual_position);
	    }
	}
      while(myepos_read.number[0].actual_position != (int) pos[1] ||
	    myepos_read.number[1].actual_position != (int) pos[2] ||
	    myepos_read.number[2].actual_position != (int) pos[3] ||
	    myepos_read.number[3].actual_position != (int) pos[4] ||
	    myepos_read.number[4].actual_position != (int) pos[5] );
	    
    }



  for(id=1;id<7;id++)
    {
      shutdown(id);	
    }	
  
  
  canHWEnd();
  
  printf("end\n");
  return 0;
}



