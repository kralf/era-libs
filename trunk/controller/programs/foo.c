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

#define MAXERROR 31

typedef struct ERROR{
	long int no;
	char *msg;
} ERROR;

static ERROR error[MAXERROR] = {
	
		{ 0x00000000, "No error.\n" },
		{ 0x06020000, "Object does not exist in the object dicionary.\n" },
		{ 0x06090011, "Sub-index does not exist.\n" },											
		{ 0x05040001, "Client/server command specifier not valid or unknown.\n" },										
		{ 0x05030000, "Toggle bit not alternated.\n" },										
		{ 0x05040000, "SDO protocol timed out.\n" },										
		{ 0x05040005, "Out of memory\n" },										
		{ 0x06010000, "Unsupported access to an object.\n" },										
		{ 0x06010001, "Attempt to read a write only object.\n"  },										
		{ 0x06010002, "Attempt to write a read only object.\n"  },										
		{ 0x06040041, "Object cannot be mapped to the PDO.\n" },										
		{ 0x06040042, "The number and length of the objects to be mapped would exceed PDO length.\n" },										
		{ 0x06040043, "General parameter incompatibility reason.\n" },										
		{ 0x06040047, "General internal incompatibility reason.\n" },										
		{ 0x06060000, "Access failed due to an hardware error.\n" },										
		{ 0x06070010, "Data type does not match, length of service parameter does not match.\n" },										
		{ 0x06070012, "Data type does not match, length of service parameter too high.\n" },										
		{ 0x06070013, "Data type does not match, length of service parameter too low.\n" },										
		{ 0x06090030, "Value range of parameter exceeded (only for write access)\n" },
		{ 0x06090031, "Value of parameter written too high.\n" },										
		{ 0x06090032, "Value of parameter written too low.\n" },										
		{ 0x06090036, "Maximum value is less than minimum value.\n" },										
		{ 0x08000000, "General error.\n" },										
		{ 0x08000020, "Data cannot be transferred or stored to the application.\n" },										
		{ 0x08000021, "Data cannot be transferred or stored to the application because of local control.\n" },										
		{ 0x08000022, "Data cannot be transferred or stored to the application because of the present device state.\n"},										
		{ 0x0F00FFC0, "The device is in wrong NMT state.\n" },										
		{ 0x0F00FFBF, "The RS232 command is illegal.\n" },										
		{ 0x0F00FFBE, "The password is not correct.\n" },										
		{ 0x0F00FFBC, "The device is not in service mode.\n" },										
		{ 0x0F00FFB9, "Error Node-ID.\n" },										

	}; 
  
int main(void) 
{
	int i;
	
	for(i=0;i<MAXERROR;i++) if(error[i].no == 0x08000000) printf("0x%08lX\n%s\n", error[i].no, error[i].msg );
	
	return 0;
}


		