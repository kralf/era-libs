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
	
	canHWInit();
	
	fault_reset(id);
	shutdown(id);
	//enable_operation(id);	
	
	set_maximum_following_error(id, 0x80002000);
	get_maximum_following_error(id);
	printf("value: 0x%08X\n", myepos_read.number[id-1].maximum_following_error);

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


