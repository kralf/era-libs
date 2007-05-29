/*	Trayectory Generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    20.5.2007
 */




#include"kin.h"
#include"libserial.h"  
#include<stdio.h>
#include<stdlib.h>
#include<math.h>

//#include"kin2serial.h"
#include"trajectory.h"


#define MCI_MAX_VIA_POINTS  50
#define MCI_MAX_VEL_INTERVALS  500
//#define MCI_MAX_VEL  2  // [rad/s]

/** Limits of angular velocities according to BlueBotics specification */
const float theta_vel_max[] = {M_PI*13/36, M_PI*2/5, M_PI*5/12, M_PI*5/12,     M_PI}; //[rad/s]



void mci_slope(float via_points[][6], 
	       int number_of_via_points, 
	       float m[][6])
{
  
  int i,p;
  float delta, alpha, beta, tau;

  for(p=0; p<6; p++)  
    {
      m[0][p] = 0;
      m[number_of_via_points-1][p] = 0;
      for(i=1; i<number_of_via_points-1; i++)
	m[i][p] = via_points[i+1][p] - via_points[i-1][p];
      
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
 
	


float mci_eval(float p_a, 
	       float p_b, 
	       float m_a,
	       float m_b,
	       float t)
{
  float eval;
  eval = (  (2*t*t*t - 3*t*t + 1)*p_a + (t*t*t - 2*t*t + t)*m_a +
	    (-2*t*t*t + 3*t*t   )*p_b + (t*t*t - t*t      )*m_b  );
  return eval;
}





int mci_theta_vel(float via_points[][6], 
		   float via_points_time[], 
		   int number_of_via_points, 
		   float m[][6],
		   float theta_vel[][6],
		   int* number_vel_intervals,
		   float dt)
{

  int i,j,k;
  int error = 0;
  
  float dj;
  int number_of_steps;
  int step = 0;

  float tool_a[6];
  float tool_b[6];
  float theta_a[6];
  float theta_b[6];


  for(i=0; i<(number_of_via_points-1); i++)
    {

      //  printf("testvia_points_time[i]%f\n", via_points_time);  
      number_of_steps = (int) (via_points_time[i] /dt);
      dj = 1/(float)number_of_steps;
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
	    error = 1;
	  if(inverse_kinematics(  tool_a, theta_a  )) 
	    error = 1;
	  // theta_print_rad( theta_a  );

	  for(k=0; k<6; k++)
	    {
	      //printf("theta_b_k %f\n", theta_b[k]);
	      theta_vel[step][k] = (  theta_b[k] - theta_a[k]  ) /dt;
	      if( !( theta_vel[step][k] < theta_vel_max[k] && theta_vel[step][k] > -theta_vel_max[k] )   )
		{
		  printf("ERROR mci: Maximumm velocity exceeded: vel[%i]=%f\n\n", i+1, theta_vel[step][k]);
		  error = 1;
		}
	    }
	  step++;
	}
    }
  *number_vel_intervals = step;
  return error;
}


int mci(float via_points[][6],
	float via_points_time[],
	int number_of_via_points,
	float theta_vel[][6],
	int* number_vel_intervals,
	float dt)
{

  if(number_of_via_points > MCI_MAX_VIA_POINTS)
    {
      printf("ERROR mci: Number of via points exceeds maximum\n\n");
      return 1;
    }
  
  float m[MCI_MAX_VIA_POINTS][6];
  int i;
  

  mci_slope(via_points, number_of_via_points, m);

  for(i=0; i<6; i++)
  printf("m: %f, %f, %f, %f, %f, %f, %f, %f, %f, %f,  %f, %f, %f, %f, %f, %f\n", 
     m[0][i], m[1][i], m[2][i], m[3][i], m[4][i], m[5][i], m[6][i], m[7][i], m[8][i], m[9][i], 
     m[10][i], m[11][i], m[12][i], m[13][i], m[14][i], m[15][i]);
  

  if( mci_theta_vel( via_points,  via_points_time,  number_of_via_points, 
		     m, theta_vel, number_vel_intervals, dt)  )
    return 1;

  

  return 0;
}

	


    
  
/*
float time_between_via_points( float via_points[][], 
			       int number_of_via_points,
			       float via_points_time[], 
			       float vel)
{
  int i;
  float dist;
  for(i=0; i<number_of_via_points-1; i++)
    {
      dist = sqrt( sqr(via_points[i][0] - via_points[i+1][0]) +
		   sqr(via_points[i][1] - via_points[i+1][1]) +
		   sqr(via_points[i][2] - via_points[i+1][2]) );
      via_points_time[i] = dist / vel;
    }
}*/
