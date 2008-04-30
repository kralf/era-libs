/*	Test-file for serial EPOS-communication
 *	V0.1
 * 	� Marc Rauer ETHZ	marc.rauer@gmx.de
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
	int id=2;
	int i;
	
	canHWInit();
	
	fault_reset(id);
	shutdown(id);
	enable_operation(id);
	set_mode_of_operation(id,1);
	set_output_current_limit(id, 500);
	shutdown(id);		
	enable_operation(id);


		set_target_position(id,10000);

		activate_position(id);

	
	printf("Testausgabe vor : 0x%08lX %s\n",myepos_read.number[id-1].error.serial.code, 			
				myepos_read.number[id-1].error.serial.msg); 			
	
	set_maximum_following_error(id, 0x00000003);
	get_maximum_following_error(id);
	printf("value: 0x%08X\n", myepos_read.number[id-1].maximum_following_error);

	printf("Testausgabe nach: 0x%08lX %s\n",myepos_read.number[id-1].error.serial.code, 			
				myepos_read.number[id-1].error.serial.msg); 			

	printf("-----------------------------------------------------------------------------\n");

	get_error(id);


	printf("Testausgabe vor:\ncount: %d\nreg: 0x%02X\n",myepos_read.number[id-1].error.device.count, 			
				myepos_read.number[id-1].error.device.reg); 			
	for(i=0;i<5;i++)
	{
		printf("history[%d]: code: 0x%02X\treg: 0x%02X\tmsg: %s\n", i, myepos_read.number[id-1].error.device.history[i].code
		, myepos_read.number[id-1].error.device.history[i].reg, myepos_read.number[id-1].error.device.history[i].msg);	
	}


	fault_reset(id);
	get_error(id);


	printf("Testausgabe nach:\ncount: %d\nreg: 0x%02X\n",myepos_read.number[id-1].error.device.count, 			
				myepos_read.number[id-1].error.device.reg); 			
	for(i=0;i<5;i++)
	{
		printf("history[%d]: code: 0x%02X\treg: 0x%02X\tmsg: %s\n", i, myepos_read.number[id-1].error.device.history[i].code
		, myepos_read.number[id-1].error.device.history[i].reg, myepos_read.number[id-1].error.device.history[i].msg);	
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


