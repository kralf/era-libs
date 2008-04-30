#include<era_kinematics.h>
#include<math.c>

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

  v_beta1.x = -sin(*beta_1);
  v_beta1.y = cos(*beta_1);
  v_beta1.z = 0;

  v_sp.x = v_target->x - a5*v_beta1.x ;
  v_sp.y = v_target->y - a5*v_beta1.y ; 
  v_sp.y = v_target->z - (a3+a4+a5) - a5*v_beta1.z;

  v_normal.x = ( v_sp.y * v_beta1.z  -  v_sp.z * v_beta1.y );
  v_normal.y = ( v_sp.z * v_beta1.x  -  v_sp.x * v_beta1.z );
  v_normal.z = ( v_sp.x * v_beta1.y  -  v_sp.y * v_beta1.x );

  //theta->theta_1 = atan(v_normal.y / v_normal.x);
  theta->theta_1 = *beta_1;
  theta->theta_2 = atan( v_normal.z*v_normal.z / 
			 ( v_normal.x*v_normal.x + v_normal.y*v_normal.y) );

  x =    v_sp.x * sin(theta->theta_2)*cos(theta->theta_1) 
      +  v_sp.y * sin(theta->theta_2)*sin(theta->theta_1)
      -  v_sp.z * cos(theta->theta_2);
  y = -  v_sp.x * sin(theta->theta_1)  +  v_sp * cos(theta->theta_1);
  
  c4 = (x^2+y^2- a3^2 - a4^2) / (2*a4*a3);
  theta->theta_4 = atan(sqrt(1-c4^2)/c4);
  theta->theta_3 = atan(y/x) - atan( (a4*sqrt(1-c4^2)) / (a3+a4*c4) );

  // theta->theta_5 = pi/2 - theta->theta_3 - theta->theta_3;
  theta->theta_6 = theta->theta_2 + beta_2;

}
