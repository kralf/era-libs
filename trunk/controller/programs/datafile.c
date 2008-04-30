/*	Write position and velocity-values to file
 *	V0.1
 * 	(C) Marc Rauer ETHZ	marc.rauer@gmx.de
 * 	Last change: 05/30/07
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include "kin.h"



int write_datafile(char *filename, float *tool, float *theta)
{
	FILE *fd;
	char buffer[255];
	int no=0,ret=0,i;

	fd = fopen(filename,"w+t");

	if(fd == NULL) {
		printf("Error while opening the file for writing!\n");
	return 0;
	}
	
	for(i=0;i<6;i++) no += sprintf(buffer+no, "%f ", tool[i]);
	for(i=0;i<7;i++) no += sprintf(buffer+no, "%f ", theta[i]);
	// add additional data to write in file here! 

	no += sprintf(buffer+no, "\n");

	ret = fwrite(buffer, sizeof(char), no, fd);

	if(ret != no) {
		printf("Error while writing file! Only %d chars of %d have been written.\n", ret, no);
		return ret;
	}

	//printf("Wrote %d char(s): |%s|\n", ret,buffer);	
	return ret;
}


int main(int argc, char **argv)
{
	int ret=0;

	system("clear");

	float tool[6] = {-1.10,2.20,-3.30,4.40,-5.50,6.60};
	float theta[7] = {0.10,0.20,0.30,0.40,0.50,0.60,0.701};


	if(argc == 1) {
		printf("No datafile specified!\nUse %s 'filename' .\n", argv[0]);
 		return 0;
	}	
	printf("datafile: %s \n", argv[1]);

	ret = write_datafile(argv[1], tool, theta);

	return 0;
}
