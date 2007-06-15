/*	Trayectory Generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    20.5.2007
 */




#include"libkin.h"
#include"libserial.h"  
#include<stdio.h>
#include<stdlib.h>
#include<math.h>
#include <string.h>

//#include"kin2serial.h"
#include"libtrajectory.h"


#define MCI_MAX_VIA_POINTS  50
#define MCI_MAX_VEL_INTERVALS  500
//#define MCI_MAX_VEL  2  // [rad/s]

/** Limits of angular velocities according to BlueBotics specification */
//const double theta_vel_max[] = {M_PI*13/36, M_PI*2/5, M_PI*5/12, M_PI*5/12,     M_PI}; //[rad/s]
const double theta_vel_max[] = {0.3, 0.3, 0.3, 0.3,     0.5}; // testing[rad/s]

/*
void trajectory_auto_angle(double via_points[][6], 
			   int number_of_via_points)
{
  int i;
  for(i=0; i<number_of_via_points; i++)
    {
      if(via_points[i][3] == 999)
	via_points[i][3] = -180/M_PI*tan(via_points[i][0] /via_points[i][1]);
      printf("angle auto: %f \n", via_points[i][3]);
    }
}
*/



int read_scriptfile(char *filename, double via_points[][7], int *number_of_via_points)
{
	FILE *fd;
	char buffer[255];
	double val[7];
	int ret=0,no_lines=0,no_commment=0;
	int i;

	fd = fopen(filename,"r");

	if(fd == NULL) {
		printf("File not found!\n");
	return 1;
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
			return 1;
		}
		for(i=0; i<7; i++)
		  {
		    via_points[no_lines][i] = val[i];
		  }

		no_lines++;
	}
	*number_of_via_points = no_lines;
	return 0;
}



void mci_slope(double via_points[][7], 
	       int number_of_via_points, 
	       double m[][6])
{
  
  int i,p;
  double delta, alpha, beta, tau;

  for(p=0; p<6; p++)  
    {
      m[0][p] = 0;
      m[number_of_via_points-1][p] = 0;
      for(i=1; i<number_of_via_points-1; i++)
	m[i][p] = (via_points[i+1][p] - via_points[i-1][p])/2;
        //      m[i][p] = via_points[i+1][p] - via_points[i-1][p];
      
      for(i=0; i<number_of_via_points-1; i++)
	{
	  delta = via_points[i+1][p] - via_points[i][p];
	  if(delta==0)
	    {
	      m[i][p]   = 0;
	      m[i+1][p] = 0;
	    }
	  else
	    {
	      alpha = m[i][p] / delta;
	      beta  = m[i+1][p] / delta;
	      if(alpha + beta -2 > 0 &&
		 2*alpha + beta -3 > 0 &&
		 alpha + 2*beta -3 > 0 &&
		 alpha -(2*alpha+beta-3)*(2*alpha+beta-3)/(alpha+beta-2)/3 < 0 )
		{
		  tau = 3/sqrt(alpha*alpha + beta*beta);
		  m[i][p]   = tau*alpha*delta;
		  m[i+1][p] = tau*beta*delta;
		}
	    }
	}
    }	 

}
 
	


double mci_eval(double p_a, 
	       double p_b, 
	       double m_a,
	       double m_b,
	       double t)
{
  double eval;
  eval = (  (2*t*t*t - 3*t*t + 1)*p_a + (t*t*t - 2*t*t + t)*m_a +
	    (-2*t*t*t + 3*t*t   )*p_b + (t*t*t - t*t      )*m_b  );
  return eval;
}





int mci_theta_vel(double via_points[][7], 
		  //double via_points_time[], 
		  int number_of_via_points, 
		  double m[][6],
		  double theta_vel[][6],
		  int* number_vel_intervals,
		  double dt,
		  int kin_err[][3])
{

  int i,j,k;
  int error = 0;
  
  double dj;
  int number_of_steps;
  int step = 0;

  double tool_a[6];
  double tool_b[6];
  double theta_a[6];
  double theta_b[6];


  for(i=0; i<(number_of_via_points-1); i++)
    {

      //  printf("testvia_points_time[i]%f\n", via_points_time);  
      number_of_steps = (int) (via_points[i][6] /dt);
      dj = 1/(double)number_of_steps;
      printf("number of steps %i\n", number_of_steps);
      for(j=0; j<number_of_steps; j++)
	{
	  for(k=0; k<6; k++)
	    {
	      tool_b[k]  = mci_eval(  via_points[i][k], via_points[i+1][k], m[i][k], m[i+1][k], (j+1)*dj  ); 
	      tool_a[k]  = mci_eval(  via_points[i][k], via_points[i+1][k], m[i][k], m[i+1][k], (j)*dj  ); 
	      //printf("tool_a_k %f\n", tool_a[k]);
	      //printf("tool_b_k %f\n", tool_b[k]);
	      //printf("dj %f steps %i\n", dj, number_of_steps);

	    }
	  if(inverse_kinematics(  tool_b, theta_b  )) 
	    {
	      error = 1;
	      kin_err[step][1] = 1;
	    }
	  else
	    kin_err[step][1] = 0;

	  //printf("theta2 %f\n", theta_b[1]);
	  if(inverse_kinematics(  tool_a, theta_a  )) 
	    error = 1;
	  //printf("theta2 %f\n", theta_a[1]);
	  // theta_print_rad( theta_a  );

	  kin_err[step][2] = 0; 
	  for(k=0; k<6; k++)
	    {
	      //printf("theta_b_k %f\n", theta_b[k]);
	      theta_vel[step][k] = (  theta_b[k] - theta_a[k]  ) /dt;
	      if( !( theta_vel[step][k] < theta_vel_max[k] && theta_vel[step][k] > -theta_vel_max[k] )   )
		{
		  printf("ERROR mci: Maximumm velocity exceeded: vel[%i]=%f\n\n", k+1, theta_vel[step][k]);
		  kin_err[step][2] = 1; 
		  error = 1;
		}
	      
	    }
	  kin_err[step][0] = i+1; 
	  step++;

	}
    }
  *number_vel_intervals = step;
  return error;
}


int mci(double via_points[][7],
	//double via_points_time[],
	int number_of_via_points,
	double theta_vel[][6],
	int* number_vel_intervals,
	double dt,
	int kin_err[][3])
{

  if(number_of_via_points > MCI_MAX_VIA_POINTS)
    {
      printf("ERROR mci: Number of via points exceeds maximum\n\n");
      return 1;
    }
  
  double m[MCI_MAX_VIA_POINTS][6];
  //  int i;
  

  mci_slope(via_points, number_of_via_points, m);
  /*
  for(i=0; i<6; i++)
  printf("m: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, %f\n", 
     m[0][i], m[1][i], m[2][i], m[3][i], m[4][i], m[5][i], m[6][i], m[7][i], m[8][i], m[9][i], 
     m[10][i], m[11][i], m[12][i], m[13][i], m[14][i], m[15][i]);
  */

  if( mci_theta_vel( via_points,  number_of_via_points, 
		     m, theta_vel, number_vel_intervals, dt, kin_err)  )
    return 1;

  

  return 0;
}

	


    
  
/*
double time_between_via_points( double via_points[][], 
			       int number_of_via_points,
			       double via_points_time[], 
			       double vel)
{
  int i;
  double dist;
  for(i=0; i<number_of_via_points-1; i++)
    {
      dist = sqrt( sqr(via_points[i][0] - via_points[i+1][0]) +
		   sqr(via_points[i][1] - via_points[i+1][1]) +
		   sqr(via_points[i][2] - via_points[i+1][2]) );
      via_points_time[i] = dist / vel;
    }
}*/
