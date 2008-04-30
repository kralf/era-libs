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



int read_scriptfile(char *filename, double via_points[][7])
{
	FILE *fd;
	char buffer[255];
	double val[7];
	int ret=0,no_lines=0,no_commment=0;
	int i;

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

		ret = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf", &val[0], &val[1], &val[2], &val[3], 
			     &val[4], &val[5], &val[6]);

		if(ret != 7) {
			printf("Error in scriptfile maybe at line %d!\n", (no_lines+1)+no_commment);
			return 0;
		}
		for(i=0; i<7; i++)
		  {
		    via_points[no_lines][i] = val[i];
		  }

		no_lines++;
	}
	return no_lines;
}


int main(int argc, char **argv)
{
  double path[100][7];

  int no_points;	
  int n=0;
  int i;

	system("clear");

	if(argc == 1) {
		printf("No scriptfile specified!\nUse %s 'filename' .\n", argv[0]);
 		return 0;
	}	
	printf("scriptfile: %s \n", argv[1]);


	no_points = read_scriptfile(argv[1], path);


	for(n=0;n<no_points;n++) {
	  for(i=0; i<7; i++)
	    printf(" %lf\t", path[n][i]);
	  printf("\n");
	}	
	return 0;
}
