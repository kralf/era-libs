/*	Read Coordinates form file
 *	V0.1
 * 	(C) Marc Rauer ETHZ	marc.rauer@gmx.de
 * 	Last change: 05/23/07
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>


#include "kin.h"



int read_scriptfile(char *filename, t_target *coord, double *vel)
{
	FILE *fd;
	char buffer[255];
	double val[6];
	int ret=0,no_lines=0,no_commment=0;

	fd = fopen(filename,"r");

	if(fd == NULL) {
		printf("File not found!\n");
	return 0;
	}
	
	while(fgets(buffer, 255, fd) != NULL)	 
	{
		if(buffer[0] == '/' && buffer[1] == '/') {
			no_commment++;
			continue;
		}

		ret = sscanf(buffer, "%lf %lf %lf %lf %lf %lf", &val[0], &val[1], &val[2], &val[3], &val[4], &val[5]);

		if(ret != 6) {
			printf("Error in scriptfile maybe at line %d!\n", (no_lines+1)+no_commment);
			return 0;
		}

		coord[no_lines].x = val[0];
		coord[no_lines].y = val[1];
		coord[no_lines].z = val[2];
		coord[no_lines].beta1 = val[3];
		coord[no_lines].beta2 = val[4];
		vel[no_lines] = val[5];

		no_lines++;
	}
	return no_lines;
}


int main(int argc, char **argv)
{
	t_target coord[10];
	double vel[10];
	int no_points;	
	int n=0;

	system("clear");

	if(argc == 1) {
		printf("No scriptfile specified!\nUse %s 'filename' .\n", argv[0]);
 		return 0;
	}	
	printf("scriptfile: %s \n", argv[1]);


	no_points = read_scriptfile(argv[1], coord, vel);


	for(n=0;n<no_points;n++) {

		printf("\tcoord[%d].x=[%f]\n",n , coord[n].x);
		printf("\tcoord[%d].y=[%f]\n",n , coord[n].y);
		printf("\tcoord[%d].z=[%f]\n",n , coord[n].z);
		printf("\tcoord[%d].beta1=[%f]\n",n , coord[n].beta1);
		printf("\tcoord[%d].beta2=[%f]\n",n , coord[n].beta2);
		printf("\tvel[%d]=[%f]\n\n",n , vel[n]);	
	}	
	return 0;
}
