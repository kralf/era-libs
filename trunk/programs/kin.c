/*	Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    1.5.2007
 */


#include"kin.h"
#include<math.h>
#include<stdio.h>


double sqr(double x) 
{return x*x;}


void forward_kinematics(t_target* target, t_theta* th)
{
  double a3 = 23.05;
  double a4 = 22.4;
  double a5 = 18.8;

  target->x = -a4*cos(th->theta4)*sin(th->theta1)*sin(th->theta3)-a4*sin(th->theta4)*sin(th->theta1)*cos(th->theta3)
              +a3*cos(th->theta1)*sin(th->theta2)*cos(th->theta3)
              +a4*cos(th->theta4)*cos(th->theta1)*sin(th->theta2)*cos(th->theta3)
              -a3*sin(th->theta1)*sin(th->theta3)-a4*sin(th->theta4)*cos(th->theta1)*sin(th->theta2)*sin(th->theta3)
              -sin(th->theta1)*a5;
  
  target->y = cos(th->theta1)*a5+a3*sin(th->theta1)*sin(th->theta2)*cos(th->theta3)
             +a4*sin(th->theta4)*cos(th->theta1)*cos(th->theta3)
             +a4*cos(th->theta4)*cos(th->theta1)*sin(th->theta3)+a3*cos(th->theta1)*sin(th->theta3)
             +a4*cos(th->theta4)*sin(th->theta1)*sin(th->theta2)*cos(th->theta3)
             -a4*sin(th->theta4)*sin(th->theta1)*sin(th->theta2)*sin(th->theta3);
  
  target->z = -cos(th->theta2)*cos(th->theta3)*cos(th->theta4)*a4+cos(th->theta2)*sin(th->theta3)*sin(th->theta4)*a4
             -cos(th->theta2)*cos(th->theta3)*a3+a3+a4+a5;
  
  target->beta1 = th->theta1;
  target->beta2 = th->theta6 - th->theta2;


}


void inverse_kinematics(t_target* target, t_theta* theta)
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

  v_beta1.x = -sin(target->beta1);
  v_beta1.y = cos(target->beta1);
  v_beta1.z = 0;


  v_sp.x = target->x - a5*v_beta1.x ;
  v_sp.y = target->y - a5*v_beta1.y ; 
  v_sp.z = target->z - (a3+a4+a5) - a5*v_beta1.z;
  //v_sp.x = target->x ;
  //v_sp.y = target->y ;
  //v_sp.z = target->z - (a3+a4+a5);


  v_normal.x = ( v_sp.y * v_beta1.z  -  v_sp.z * v_beta1.y );
  v_normal.y = ( v_sp.z * v_beta1.x  -  v_sp.x * v_beta1.z );
  v_normal.z = ( v_sp.x * v_beta1.y  -  v_sp.y * v_beta1.x );



  //  theta->theta1 = atan2(v_normal.y , v_normal.x);
  theta->theta1 = target->beta1;
  theta->theta2 = atan2( (v_normal.z) , 
  		  sqrt( sqr(v_normal.x)+sqr(v_normal.y) ) );


  x =    v_sp.x * sin(theta->theta2)*cos(theta->theta1)  
      +  v_sp.y * sin(theta->theta2)*sin(theta->theta1)  
      -  v_sp.z * cos(theta->theta2);
  y = -  v_sp.x * sin(theta->theta1)  +  v_sp.y * cos(theta->theta1);
  
  c4 = (sqr(x)+sqr(y)- sqr(a3) - sqr(a4)) / (2*a4*a3);
  theta->theta4 = atan2(sqrt(1-sqr(c4)),c4);
  theta->theta3 = atan2(y,x) - atan2( (a4*sqrt(1-sqr(c4))) , (a3+a4*c4) );

  // theta->theta5 = pi/2 - theta->theta3 - theta->theta3;
  theta->theta6 = theta->theta2 + target->beta2;

}
