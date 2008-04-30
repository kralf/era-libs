/*	Connecting Kinematic system model for BlueBotics ERA-5/1
 *      with libserial
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    3.5.2007
 */

#ifndef _KIN2SERIAL_H
#define _KIN2SERIAL_H



#include"kin2serial.h"

#include"kin.h"
#include"libserial.h"  
#include<stdio.h>
#include<stdlib.h>
#include<math.h>




float max(float m1, float m2)
{
  if(m1>m2)
    return m1;
  else
    return m2;
}

/*
float abs(float a)
{
  if(a>0)
    return a;
  else
    return -a;
}*/

void kin2s_position_mode_calc_vel(float pos[], float pos_old[], float vel[], float vel_max)
{

  float dpos[6];
  int i=0;
  for(i=0;i<=5;i++)
    dpos[i] = abs( (pos[i] - pos_old[i]) );

  // maximum position difference:
  float max_dpos = max(max(dpos[1], dpos[2]),
                       max(dpos[3], dpos[4]));

  float i_gear[] = { 0.02,        0.02,        0.02,        0.02,        0.005, 0.01};
  float i_arm[]  = { 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1,     1};

  for(i=0;i<=5;i++)
    vel[i] = vel_max * (dpos[i]/max_dpos) / (i_gear[i]*i_arm[i]);


}

void kin2s_position_mode_set(float pos[], float vel[])
{
 
  int id=0;
  for(id=1;id<7;id++)
    {
      
      set_profile_velocity( id, vel[id-1]);
      set_target_position(  id, pos[id-1]);
      activate_position(    id);
    }

  
}



void kin2s_position_mode_init()
{
  int id=0;
  for(id=1;id<7;id++)
    {
      fault_reset(id);
      shutdown(id);
      enable_operation(id);	
      set_mode_of_operation(id,1);
      shutdown(id);		
      enable_operation(id);	
    }

 
  t_target target;
  t_theta th;
}


void kin2s_init(float theta[], float target[])
{


  target_init_starting_values( target );
  inverse_kinematics( target, theta);
  target_print( target );
  theta_print_rad( theta );
  theta_rad_to_tiks( theta );
  theta_print_tiks( theta );

  /*
  theta_init_start_tiks( &th );
  th_rad = theta_tiks_to_rad( &th_tiks);


  forward_kinematics( &target_rad2, &th_rad);




  th_tiks = theta_rad_to_tiks( &th_rad);

  theta_print_rad(&th_rad);
  theta_print_tiks(&th_tiks);
  target_print(&target_rad);


  th_tiks = theta_rad_to_tiks( &th_rad);

  //  theta_print_rad(&th_rad);
  //  theta_print_tiks(&th_tiks);
  target_print(&target_rad2);

  */
}


void theta_init_start_tiks(float theta[])
{
  theta[0] = -30000;
  theta[1] =  49000;
  theta[2] =  0;
  theta[3] =  50000;
  theta[4] = -35000;
  //  long int home_offset[] = {50000,50000,50000,50000,180000,30000}; // starting position
  //  long int zero_offset[] = {80000,1000,50000,0,215000,30000};      // down zero position
}



void target_init_starting_values(float target[])
{
  target[0]     =  20.932410; //20.931822;
  target[1]     =  23.874095; //23.874062;
  target[2]     =  23.513540; //23.513273;
  target[3]     =  -0.204886; //-0.204880;
  target[4]     =  -1.466077; //-0.366508;
}



void theta_print_rad(float theta[])
{

  printf("Joint Angles\n");
  printf("  theta1: %f°\n", theta[0]*180/M_PI);
  printf("  theta2: %f°\n", theta[1]*180/M_PI);
  printf("  theta3: %f°\n", theta[2]*180/M_PI);
  printf("  theta4: %f°\n", theta[3]*180/M_PI);
  printf("  theta6: %f°\n", theta[4]*180/M_PI);
  printf("  Gripper: %f\n", theta[5]);

  
}

void theta_print_tiks(float theta[])
{

  printf("Joint Angles\n");
  printf("  theta1: %f tiks\n", theta[0]);
  printf("  theta2: %f tiks\n", theta[1]);
  printf("  theta3: %f tiks\n", theta[2]);
  printf("  theta4: %f tiks\n", theta[3]);
  printf("  theta6: %f tiks\n", theta[4]);
  printf("  Gripper: %f tiks\n", theta[5]);
  
}


void target_print(float target[])
{
  printf("End Effector state\n");
  printf("  x:     %f\n", target[0]);
  printf("  y:     %f\n", target[1]);
  printf("  z:     %f\n", target[2]);
  printf("  beta1: %f° %f rad \n", target[3]*180/M_PI, target[3]);
  printf("  beta2: %f° %f rad \n", target[4]*180/M_PI, target[4]);
  
}


void theta_rad_to_tiks(float theta[])
{
  int i=0;

  float enc_rev[]={500,500,500,500,500,500};
  float i_gear[]={0.02,0.02,0.02,0.02,0.005,0.01};
  float i_arm[]={ 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};
  long int zero_offset[] = {80000,1000,50000,0,215000,30000};      // down zero position
  long int home_offset[] = {50000,50000,50000,50000,180000,30000}; // starting position

  for(i=0; i<=4; i++)
    theta[i] = theta[i]/(2*M_PI)/(i_gear[i]*i_arm[i])*(enc_rev[i]*4) + zero_offset[i] - home_offset[i];
 

}


void theta_tiks_to_rad(float theta[])
{
  int i=0;
  float enc_rev[]={500,500,500,500,500,500};
  float i_gear[]={0.02,0.02,0.02,0.02,0.005,0.01};
  float i_arm[]={ 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};
  long int zero_offset[] = {80000,1000,50000,0,215000,30000};      // down zero position
  long int home_offset[] = {50000,50000,50000,50000,180000,30000}; // starting position

  for(i=0; i<=4; i++)
    theta[i] = (theta[i] - zero_offset[i] +  home_offset[i] ) *(2*M_PI)*(i_gear[i]*i_arm[i])/(enc_rev[i]*4);  
 
}



#endif
