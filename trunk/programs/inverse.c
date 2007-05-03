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

#include "libserial.h"  
#include "kin.h"



int main(void) 
{


  t_theta th_rad, th_tiks;
  t_target target_rad, target_rad2;

  //  target_init_starting_values( &target_rad);

  theta_init_start_tiks( &th_tiks);
  th_rad = theta_tiks_to_rad( &th_tiks);

  //  inverse_kinematics( &target_rad, &th_rad);

  th_tiks = theta_rad_to_tiks( &th_rad);

  theta_print_rad(&th_rad);
  theta_print_tiks(&th_tiks);
  target_print(&target_rad);

  forward_kinematics( &target_rad2, &th_rad);
  th_tiks = theta_rad_to_tiks( &th_rad);

  //  theta_print_rad(&th_rad);
  //  theta_print_tiks(&th_tiks);
  target_print(&target_rad2);




  //  system("clear");
  printf("Testprogram ERA V0.1\n--------------------\nPlease wait for init...\n\n");
  
  //  canHWInit();



    /*
	int id=0;
	float pos,vel,v=0,x=0;

	float enc_rev[]={0,500,500,500,500,500,500};
	float i_gear[]={0,0.02,0.02,0.02,0.02,0.005,0.01};
	float i_arm[]={0, 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};
	float v_max[]={0,20,20,20,20,20,20};
	float x_min[]={0,-10,-10,-10,-10,-90,-10};
	float x_max[]={0,10,10,10,10,90,10};


	system("clear");
	printf("Testprogram ERA V0.1\n--------------------\nPlease wait for init...\n\n");
	
	canHWInit();
	
	id = 2;
	
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
		printf("(x) Exit\nNode-ID: ");
		if(scanf("%d",&id) == 0) break;
		if(id < 1 || id > 6) continue;

		printf("Geschwindigkeit U/min: ");
		if(scanf("%e",&v) == 0) break;
		if(v < 0 || v > v_max[id]) continue;
				
		printf("Winkel �: ");
		if(scanf("%e",&x) == 0) break;
		if(x < x_min[id] || x > x_max[id]) continue;

		pos = x/(360)/(i_gear[id]*i_arm[id])*(enc_rev[id]*4);
		vel = v/(i_gear[id]*i_arm[id]);
		
		printf("Velocity: %f\n",vel);
		printf("pos: %f\n\n",pos);
		
		set_profile_velocity(id,vel);
		set_target_position(id,pos);
		activate_position(id);
		//get_statusword(id);	
		
		do
		{
			get_actual_position(id);
			get_current_actual_value(id);
			
			printf("x: soll: %d\tist: %d\n", (int) pos, myepos_read.number[id-1].actual_position);
		}
		while(myepos_read.number[id-1].actual_position != (int) pos); 	
	}
	

	for(id=1;id<7;id++)
	{
		shutdown(id);	
	}	

		
	canHWEnd();
    */
	printf("end\n");
	return 0;
}



