/*	Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    1.5.2007
 */


#include"kin.h"
#include<math.h>
#include<stdlib.h>
#include<stdio.h>


double sqr(double x) 
{return x*x;}



void theta_print_rad(t_theta* th)
{
  printf("Joint Angles\n");
  printf("  theta1: %f°\n", th->theta1*180/M_PI);
  printf("  theta2: %f°\n", th->theta2*180/M_PI);
  printf("  theta3: %f°\n", th->theta3*180/M_PI);
  printf("  theta4: %f°\n", th->theta4*180/M_PI);
  printf("  theta6: %f°\n", th->theta6*180/M_PI);
  
}

void theta_print_tiks(t_theta* th)
{
  printf("Joint Angles\n");
  printf("  theta1: %f tiks\n", th->theta1);
  printf("  theta2: %f tiks\n", th->theta2);
  printf("  theta3: %f tiks\n", th->theta3);
  printf("  theta4: %f tiks\n", th->theta4);
  printf("  theta6: %f tiks\n", th->theta6);
  
}


void target_print(t_target* target)
{
  printf("End Effector state\n");
  printf("  x:     %f\n", target->x);
  printf("  y:     %f\n", target->y);
  printf("  z:     %f\n", target->z);
  printf("  beta1: %f°\n", target->beta1*180/M_PI);
  printf("  beta2: %f°\n", target->beta2*180/M_PI);
  
}


t_theta theta_rad_to_tiks(t_theta* th)
{
  
  t_theta th_tiks;
  float enc_rev[]={0,500,500,500,500,500,500};
  float i_gear[]={0,0.02,0.02,0.02,0.02,0.005,0.01};
  float i_arm[]={0, 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};

  th_tiks.theta1 = th->theta1/(2*M_PI)/(i_gear[1]*i_arm[1])*(enc_rev[1]*4);  
  th_tiks.theta2 = th->theta2/(2*M_PI)/(i_gear[2]*i_arm[2])*(enc_rev[2]*4);  
  th_tiks.theta3 = th->theta3/(2*M_PI)/(i_gear[3]*i_arm[3])*(enc_rev[3]*4);  
  th_tiks.theta4 = th->theta4/(2*M_PI)/(i_gear[4]*i_arm[4])*(enc_rev[4]*4);  
  th_tiks.theta6 = th->theta6/(2*M_PI)/(i_gear[6]*i_arm[6])*(enc_rev[6]*4);  
 
  return th_tiks;
}


t_theta theta_tiks_to_rad(t_theta* th)
{
  
  t_theta th_rad;
  float enc_rev[]={0,500,500,500,500,500,500};
  float i_gear[]={0,0.02,0.02,0.02,0.02,0.005,0.01};
  float i_arm[]={0, 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};

  th_rad.theta1 = th->theta1*(2*M_PI)*(i_gear[1]*i_arm[1])/(enc_rev[1]*4);  
  th_rad.theta2 = th->theta2*(2*M_PI)*(i_gear[2]*i_arm[2])/(enc_rev[2]*4);  
  th_rad.theta3 = th->theta3*(2*M_PI)*(i_gear[3]*i_arm[3])/(enc_rev[3]*4);  
  th_rad.theta4 = th->theta4*(2*M_PI)*(i_gear[4]*i_arm[4])/(enc_rev[4]*4);  
  th_rad.theta6 = th->theta6*(2*M_PI)*(i_gear[6]*i_arm[6])/(enc_rev[6]*4);  
 
  return th_rad;
}

void theta_init_start_tiks(t_theta* th)
{
  th->theta1 = -30000;
  th->theta2 = 49000;
  th->theta3 = 0;
  th->theta4 = 50000;
  th->theta6 = -35000;
  //  long int home_offset[] = {0,50000,50000,50000,50000,180000,30000}; // starting position
  //  long int zero_offset[] = {0,80000,1000,50000,0,215000,30000};      // down zero position
}


/*
void target_init_starting_values(t_target* target)
{
  target->x     = 20.931822;
  target->y     = 23.874062;
  target->z     = 23.513273;
  target->beta1 = -0.204880;
  target->beta2 = -0.366508;
}
*/


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
