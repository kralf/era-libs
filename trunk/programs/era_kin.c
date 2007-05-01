#include"era_kin.h"
#include<math.h>
#include<stdlib.h>
#include<stdio.h>


double sqr(double x) 
{return x*x;}

void inverse_kinematics(t_cartesian* v_target, double* beta_1, double* beta_2,  
			t_theta* theta)
{
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
  printf("b1 %f\n",*beta_1);
  v_beta1.x = -sin(*beta_1);
  v_beta1.y = cos(*beta_1);
  v_beta1.z = 0;
  printf("b1.x %f\n",v_beta1.x);
  printf("b1.y %f\n",v_beta1.y);
  printf("b1.z %f\n",v_beta1.z);


  //v_sp.x = v_target->x - a5*v_beta1.x ;
  //v_sp.y = v_target->y - a5*v_beta1.y ; 
  //v_sp.z = v_target->z - (a3+a4+a5) - a5*v_beta1.z;
  v_sp.x = v_target->x ;
  v_sp.y = v_target->y ;
  v_sp.z = v_target->z - (a3+a4+a5);

  printf("v_sp.x %f\n",v_sp.x);
  printf("v_sp.y %f\n",v_sp.y);
  printf("v_sp.z %f\n",v_sp.z);


  v_normal.x = ( v_sp.y * v_beta1.z  -  v_sp.z * v_beta1.y );
  v_normal.y = ( v_sp.z * v_beta1.x  -  v_sp.x * v_beta1.z );
  v_normal.z = ( v_sp.x * v_beta1.y  -  v_sp.y * v_beta1.x );



  //  theta->theta_1 = atan2(v_normal.y , v_normal.x);
  theta->theta_1 = *beta_1;
  theta->theta_2 = ( (v_normal.z) / 
  		  sqrt( sqr(v_normal.x)+sqr(v_normal.y) ) );


  x =    v_sp.x * sin(theta->theta_2)*cos(theta->theta_1)  
      +  v_sp.y * sin(theta->theta_2)*sin(theta->theta_1)  
      -  v_sp.z * cos(theta->theta_2);
  y = -  v_sp.x * sin(theta->theta_1)  +  v_sp.y * cos(theta->theta_1);
  
  c4 = (sqr(x)+sqr(y)- sqr(a3) - sqr(a4)) / (2*a4*a3);
  theta->theta_4 = atan2(sqrt(1-sqr(c4)),c4);
  theta->theta_3 = atan2(y,x) - atan2( (a4*sqrt(1-sqr(c4))) , (a3+a4*c4) );

  // theta->theta_5 = pi/2 - theta->theta_3 - theta->theta_3;
  theta->theta_6 = theta->theta_2 + *beta_2;

}
