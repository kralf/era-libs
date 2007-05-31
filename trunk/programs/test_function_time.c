/*	Test-file for serial EPOS-communication
 *	V0.1
 * 	� Marc Rauer ETHZ	marc.rauer@gmx.de
 * 	Last change: 05
/30/07
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/time.h>

#include "libserial.h"
#include "libepos.h"
#include "libepos_type.h"


int main(void) 
{
	int id=1;
	int i;	
	int n=10;

	struct timeval tp;
	struct timezone tzp;
	long t_usec, t_sec, delta_t=0, t_end=0;


	canHWInit();

	for(i=1;i<7;i++)
	{
		fault_reset(i);
		shutdown(i);
		//enable_operation(i);	
	}


	for(i=0;i<n;i++) 
	{
		gettimeofday(&tp,&tzp);
		t_usec = tp.tv_usec;
		t_sec = tp.tv_sec;
		{
			//set_velocity_mode_setting_value(id, 0x0A);
			get_actual_position(id);
			//get_error(id);

		}
		gettimeofday(&tp,&tzp);
		delta_t = (tp.tv_usec - t_usec) + (tp.tv_sec - t_sec)*1e6;
	
		printf("delta_t: %ld µs\n",delta_t);
		t_end += delta_t;
	}
	printf("average delta_t: %ld µs\n",t_end/n);
	
	canHWEnd();

	printf("end\n");
	return 0;
}
