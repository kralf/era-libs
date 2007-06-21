/*	Test-file for serial EPOS-communication
 *	V0.1
 * 	? Marc Rauer ETHZ	marc.rauer@gmx.de
 * 	Last change: 04/22/07
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <math.h>

#include "libserial.h"
#include "libepos.h"
#include "libepos_type.h"


int main(void)
{
	int id=0;

	int home_method[] = {0,17,18,17,17,18,-3};  //versch homing methodes
	long int home_offset[] = {0,45592,42443,42722,35608,176184,36300};
	long int homing_speed_switch_search[] = {0,200,200,200,200,500,200};
	long int homing_speed_zero_search[]   = {0,800,700,700,700,4000,200};

	int homing_current_threshold[]       = {0,1700,1200,1400,1400,150,150};
	int output_current_limit[]           = {0,2500,2000,2000,2000,300,300};


	canHWInit();

	for(id=1;id<7;id++)
	{
	  fault_reset(id);
	  shutdown(id);                 //Steuerung zuerst runter zum initialisieren
	  enable_operation(id);	        //Steuerung aktiv
	  set_mode_of_operation(id,6);  //modus ausw?hlen 6:homing
	  shutdown(id);	                //Damit neuer mode ?bernommen
	  enable_operation(id);	        //

		set_output_current_limit(id, output_current_limit[id]);
		set_homing_method(id, home_method[id]);
		set_home_offset(id, home_offset[id]);   //dort f?hrt er danach hin
		set_homing_current_threshold(id, homing_current_threshold[id]);
		set_homing_speed_switch_search(id,homing_speed_switch_search[id]);  //Geschw. zu Anschlag
		set_homing_speed_zero_search(id, homing_speed_zero_search[id]);     //Geschw. zur?ck
		start_homing_operation(id);
	}


	canHWEnd();

	printf("end\n");
	return 0;
}





















/*


void prtmsg(char *desc, char *msg, int no)
{
	int i;

	printf("%s: ", desc);
	for(i=0;i<no;i++)
	{
		printf("\t0x%02x", (unsigned char) msg[i]);
	}
	printf("\n");
}
*/


