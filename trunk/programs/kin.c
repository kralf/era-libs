/*	Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    28.5.2007
 */



#include"kin.h"
#include<math.h>
#include<stdio.h>



const float theta_min[]     = {    -M_PI/2+0.1,        0.1,   -M_PI/9+0.1,         0.1, -M_PI*8/9+0.1}; //[rad]
const float theta_max[]     = {  M_PI/6-0.1,   M_PI/2-0.1,    M_PI/2-0.1,  M_PI*2/3-0.1,  M_PI*8/9-0.1}; //[rad]

//const float theta_min[]     = {    -M_PI/2,        0,   -M_PI/9,         0, -M_PI*8/9}; //[rad]
//const float theta_max[]     = {     M_PI/6,   M_PI/2,    M_PI/2,  M_PI*2/3,  M_PI*8/9}; //[rad]

//const float theta_vel_max[] = { M_PI*13/36, M_PI*2/5, M_PI*5/12, M_PI*5/12,      M_PI}; //[rad/s]

const float arm_lenght[]    = { 23.05, 22.4, 18.8 };


double sqr(double x) 
{return x*x;}


void forward_kinematics(float tool[], 
			float theta[])
{

  float a3 = arm_lenght[0];
  float a4 = arm_lenght[1];
  float a5 = arm_lenght[2];

  /* this is x: */
  tool[0] = -a4*cos(theta[3])*sin(theta[0])*sin(theta[2])-a4*sin(theta[3])*sin(theta[0])*cos(theta[2])
              +a3*cos(theta[0])*sin(theta[1])*cos(theta[2])
              +a4*cos(theta[3])*cos(theta[0])*sin(theta[1])*cos(theta[2])
              -a3*sin(theta[0])*sin(theta[2])-a4*sin(theta[3])*cos(theta[0])*sin(theta[1])*sin(theta[2])
              -sin(theta[0])*a5;

  /* this is y: */
  tool[1] = cos(theta[0])*a5+a3*sin(theta[0])*sin(theta[1])*cos(theta[2])
             +a4*sin(theta[3])*cos(theta[0])*cos(theta[2])
             +a4*cos(theta[3])*cos(theta[0])*sin(theta[2])+a3*cos(theta[0])*sin(theta[2])
             +a4*cos(theta[3])*sin(theta[0])*sin(theta[1])*cos(theta[2])
             -a4*sin(theta[3])*sin(theta[0])*sin(theta[1])*sin(theta[2]);

  /* this is z: */
  tool[2] = -cos(theta[1])*cos(theta[2])*cos(theta[3])*a4+cos(theta[1])*sin(theta[2])*sin(theta[3])*a4
            -cos(theta[1])*cos(theta[2])*a3+a3+a4;//+a5;

  /* this is beta1: */
  tool[3] = theta[0];

  /* this is beta2: */
  tool[4] = theta[4] - theta[1];

  tool[3] = tool[3]*180/M_PI;
  tool[4] = tool[4]*180/M_PI;

  /* this is the gripper status: */
  tool[5] = theta[5];


}

int theta_workspacecheck(float theta[])
{
  int outside = 0;
  int i;
  for(i=0; i<=4; i++)
    {
      if( !(theta[i] >= theta_min[i] && theta[i] <= theta_max[i]) ) 
	{
	  outside = 1;
	  printf("ERROR: pose out of work space: theta%i = %f\n\n", i+1, theta[i]*180/M_PI);
	}
    }

  return outside;
}
    


void auto_beta1(float tool[])
{

  if ( tool[3] == 999 )
    {
      tool[3] = -tan(tool[0] /tool[1])*180/M_PI;
      printf("auto: beta1: %f\n", tool[3]);
    }
}





int inverse_kinematics(float tool[], 
			float theta[])  
{

  tool[3] = tool[3]/180*M_PI;
  tool[4] = tool[4]/180*M_PI;

  /* Arm parameters */
  float a3 = arm_lenght[0];
  float a4 = arm_lenght[1];
  float a5 = arm_lenght[2];

  float x;
  float y;
  float c4;

  t_cartesian v_sp;
  t_cartesian v_beta1;
  t_cartesian v_normal;

  v_beta1.x = -sin(tool[3]);
  v_beta1.y = cos(tool[3]);
  v_beta1.z = 0;


  v_sp.x = tool[0] - a5*v_beta1.x ;
  v_sp.y = tool[1] - a5*v_beta1.y ; 
  //v_sp.z = tool[2] - (a3+a4+a5) - a5*v_beta1.z;
  v_sp.z = tool[2] - (a3+a4) - a5*v_beta1.z;


  v_normal.x = ( v_sp.y * v_beta1.z  -  v_sp.z * v_beta1.y );
  v_normal.y = ( v_sp.z * v_beta1.x  -  v_sp.x * v_beta1.z );
  v_normal.z = ( v_sp.x * v_beta1.y  -  v_sp.y * v_beta1.x );


  /* this is theta1: */
  theta[0] = tool[3];

  /* this is theta2: */
  theta[1] = atan2( (v_normal.z) , 
  		  sqrt( sqr(v_normal.x)+sqr(v_normal.y) ) );


  x =    v_sp.x * sin(theta[1])*cos(theta[0])  
      +  v_sp.y * sin(theta[1])*sin(theta[0])  
      -  v_sp.z * cos(theta[1]);
  y = -  v_sp.x * sin(theta[0])  +  v_sp.y * cos(theta[0]);
  
  c4 = (sqr(x)+sqr(y)- sqr(a3) - sqr(a4)) / (2*a4*a3);

  /* this is theta4: */
  theta[3] = atan2(sqrt(1-sqr(c4)),c4);

  /* this is theta3: */
  theta[2] = atan2(y,x) - atan2( (a4*sqrt(1-sqr(c4))) , (a3+a4*c4) );

  /* this is theta5(which can not be controlled): */
  /* theta 5 = pi/2 - theta[2] - theta[3];  */

  /* this is theta6: */
  theta[4] = theta[1] + tool[4];          
  /* this is the gripper status: */
  theta[5] = tool[5];

  if ( theta[1]> -0.1 && theta[1]<0.1 )
    {
      //printf("theta2 exceeded %f\n", theta[1]);
      theta[1] = 0.1;
    }
  else
    {
      //printf("theta2 not exceeded %f\n", theta[1]);
    }

  return theta_workspacecheck( theta) ;
     
}
