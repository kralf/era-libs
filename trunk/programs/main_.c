/*	Test-file for serial EPOS-communication
 *	V0.4
 * 	© Marc Rauer ETHZ	marc.rauer@gmx.de
 * 	Last change: 04/21/07
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

#include <sched.h>
#include <sys/mman.h>
#include <asm/msr.h>

#include "libserial.h"  



int main(void) 
{
	int id=0;
	float pos,vel,v=0,x=0;

	float enc_rev[]={0,500,500,500,500,500,500};
	float i_gear[]={0,0.02,0.02,0.02,0.02,0.005,0.01};
	float i_arm[]={0,1,1,1,1,1,1};
	float v_max[]={0,20,20,20,20,20,20};
	float x_min[]={0,-90,-90,-90,-90,-90,-10};
	float x_max[]={0,90,90,90,90,90,10};


	system("clear");
	printf("Testprogram ERA V0.1\n--------------------\n\n");
	
	
	canHWInit();
	
	id = 3;
	
	//get_actual_error_register(id);
	//return 0;
	
	
	//for(id=1;id<7;id++)
	{
		fault_reset(id);
		shutdown(id);
		enable_operation(id);	
		set_mode_of_operation(id,1);
		shutdown(id);		
		enable_operation(id);	
	}	
	
	
	set_maximum_following_error(id, 2147483647);
	get_maximum_following_error(id);


	set_continous_current_limit(id, 500);
	get_continous_current_limit(id);
	set_output_current_limit(id, 510);
	get_output_current_limit(id);

		shutdown(id);		
		enable_operation(id);	

		set_profile_velocity(id,100);
		set_target_position(id,8000);
		activate_position(id);
		get_actual_position(id);

	

/*	
	
	while(1)
	{
		printf("(x) Exit\nNode-ID: ");
		if(scanf("%d",&id) == 0) break;
		if(id < 1 || id > 6) continue;

		printf("Geschwindigkeit U/min: ");
		if(scanf("%e",&v) == 0) break;
		if(v < 0 || v > v_max[id]) continue;
				
		printf("Winkel °: ");
		if(scanf("%e",&x) == 0) break;
		if(x < x_min[id] || x > x_max[id]) continue;

		pos = x/(360)/(i_gear[id]*i_arm[id])*(enc_rev[id]*4);
		vel = v/(i_gear[id]*i_arm[id]);
		
		printf("Velocity: %f\n",vel);
		printf("pos: %f\n\n",pos);
		
		set_profile_velocity(id,vel);
		set_target_position(id,pos);
		activate_position(id);
		get_statusword(id);	
		
	}
	

	for(id=1;id<7;id++)
	{
		shutdown(id);	
	}	

*/	
		
	canHWEnd();
	
	printf("end\n");
	return 0;
}

/*
	Firmware Seite 66:

	Encoder-Auflösung 500 Impulse (*4 pro Flanke)
	
	Geschwindigkeit in Umdrehungen pro Minute
	
	
	Strom: Achse id
	
	
	3:	400	410
	

*/




