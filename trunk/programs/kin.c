/*	Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    20.5.2007
 */


#include"kin.h"
#include<math.h>
#include<stdio.h>


double sqr(double x) 
{return x*x;}


void forward_kinematics(float target[], float theta[])
{

  double a3 = 23.05;
  double a4 = 22.4;
  double a5 = 18.8;

  target[0] = -a4*cos(theta[3])*sin(theta[0])*sin(theta[2])-a4*sin(theta[3])*sin(theta[0])*cos(theta[2])
              +a3*cos(theta[0])*sin(theta[1])*cos(theta[2])
              +a4*cos(theta[3])*cos(theta[0])*sin(theta[1])*cos(theta[2])
              -a3*sin(theta[0])*sin(theta[2])-a4*sin(theta[3])*cos(theta[0])*sin(theta[1])*sin(theta[2])
              -sin(theta[0])*a5;
  
  target[1] = cos(theta[0])*a5+a3*sin(theta[0])*sin(theta[1])*cos(theta[2])
             +a4*sin(theta[3])*cos(theta[0])*cos(theta[2])
             +a4*cos(theta[3])*cos(theta[0])*sin(theta[2])+a3*cos(theta[0])*sin(theta[2])
             +a4*cos(theta[3])*sin(theta[0])*sin(theta[1])*cos(theta[2])
             -a4*sin(theta[3])*sin(theta[0])*sin(theta[1])*sin(theta[2]);
  
  target[2] = -cos(theta[1])*cos(theta[2])*cos(theta[3])*a4+cos(theta[1])*sin(theta[2])*sin(theta[3])*a4
             -cos(theta[1])*cos(theta[2])*a3+a3+a4+a5;
  
  target[3] = theta[0];
  target[4] = theta[4] - theta[1];

  target[3] = target[3]*180/M_PI;
  target[4] = target[4]*180/M_PI;

}

void inverse_kinematics(float target[], float* theta)
{
  target[3] = target[3]/180*M_PI;
  target[4] = target[4]/180*M_PI;

  // Arm parameters
  double a3 = 23.05;
  double a4 = 22.4;
  double a5 = 18.8;

  double x;
  double y;
  double c4;

  t_cartesian v_sp;
  t_cartesian v_beta1;
  t_cartesian v_normal;

  v_beta1.x = -sin(target[3]);
  v_beta1.y = cos(target[3]);
  v_beta1.z = 0;


  v_sp.x = target[0] - a5*v_beta1.x ;
  v_sp.y = target[1] - a5*v_beta1.y ; 
  v_sp.z = target[2] - (a3+a4+a5) - a5*v_beta1.z;
  //v_sp.x = target[0] ;
  //v_sp.y = target[1] ;
  //v_sp.z = target[2] - (a3+a4+a5);


  v_normal.x = ( v_sp.y * v_beta1.z  -  v_sp.z * v_beta1.y );
  v_normal.y = ( v_sp.z * v_beta1.x  -  v_sp.x * v_beta1.z );
  v_normal.z = ( v_sp.x * v_beta1.y  -  v_sp.y * v_beta1.x );



  //  theta[0] = atan2(v_normal.y , v_normal.x);
  theta[0] = target[3];
  theta[1] = atan2( (v_normal.z) , 
  		  sqrt( sqr(v_normal.x)+sqr(v_normal.y) ) );


  x =    v_sp.x * sin(theta[1])*cos(theta[0])  
      +  v_sp.y * sin(theta[1])*sin(theta[0])  
      -  v_sp.z * cos(theta[1]);
  y = -  v_sp.x * sin(theta[0])  +  v_sp.y * cos(theta[0]);
  
  c4 = (sqr(x)+sqr(y)- sqr(a3) - sqr(a4)) / (2*a4*a3);
  theta[3] = atan2(sqrt(1-sqr(c4)),c4);
  theta[2] = atan2(y,x) - atan2( (a4*sqrt(1-sqr(c4))) , (a3+a4*c4) );

  // theta 5 = pi/2 - theta[2] - theta[2];  // this is theta 5
  theta[4] = theta[1] + target[4];          // this is theta 6
  
}
