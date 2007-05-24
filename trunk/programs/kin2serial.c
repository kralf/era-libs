/*	Connecting Kinematic system model for BlueBotics ERA-5/1
 *      with libserial
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    20.5.2007
 */


/** \file
 *  \brief 
 *   Kinematic system model 
 *
 *   Inverse and forward calculations for BlueBotics ERA-5/1 robot arm.
*/



#ifndef _KIN2SERIAL_H
#define _KIN2SERIAL_H



#include"kin2serial.h"

#include"kin.h"
#include"libserial.h"  
#include<stdio.h>
#include<stdlib.h>
#include<math.h>




const  float enc_rev[]        = {500,500,500,500,500,500};
const  float i_gear[]         = {0.02,0.02,0.02,0.02,0.005,0.01};
const  float i_arm[]          = { 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};
//const  long int zero_offset[] = {80000, 1000,50000,    0,215000,30000};      // down zero position
const  long int zero_offset[] = {80000,99000,50000,    0,145000,30000};      // down zero position
const  long int home_offset[] = {50000,50000,50000,50000,180000,30000}; // starting position
const  int sign_switch[]      = {   -1,   -1,    1,    1,     1,    1}; 




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

float kin2s_position_error(float pos_err[])
{
 return  sqrt( pos_err[0]*pos_err[0] + pos_err[1]*pos_err[1] + pos_err[2]*pos_err[2] + pos_err[3]*pos_err[3] );
}

void kin2s_position_mode_calc_vel(float pos[], float pos_old[], float vel[], float vel_max)
{

  float dpos[4];
  int i=0;
  for(i=0;i<=3;i++)
    dpos[i] = abs( (pos[i] - pos_old[i]) );

  // maximum position difference:
  float max_dpos = max(max(dpos[0], dpos[1]),
                       max(dpos[2], dpos[3]));
  /*
    float i_gear[] = { 0.02,        0.02,        0.02,        0.02,        0.005, 0.01};
      float i_arm[]  = { 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1,     1};
  */

  for(i=0;i<=3;i++)
    vel[i] = vel_max * (dpos[i]/max_dpos) / (i_gear[i]*i_arm[i]);

  vel[4] = vel_max / (i_gear[4]*i_arm[4]);
  vel[5] = vel_max / (i_gear[5]*i_arm[5]);

}

void kin2s_position_mode_set(float pos[], float vel[])
{
 
  int id=0;
  for(id=1;id<=6;id++)
    {      
      set_profile_velocity( id, vel[id-1]);
      set_target_position(  id, pos[id-1]);
    }
  for(id=1;id<=6;id++)
      activate_position(    id);  
}



void kin2s_position_mode_init()
{
  int id=0;
  for(id=1;id<=6;id++)
    {
      fault_reset(id);
      shutdown(id);
      enable_operation(id);	
      set_mode_of_operation(id,1);
      shutdown(id);		
      enable_operation(id);	
    }

 
}

/*
void kin2s_init(float theta[], float tool[])
{


  tool_init_starting_values( tool );
  inverse_kinematics( tool, theta);
  tool_print( tool );
  theta_print_rad( theta );
  theta_rad_to_tiks( theta );
  theta_print_tiks( theta );

  /-*
  theta_init_start_tiks( &th );
  th_rad = theta_tiks_to_rad( &th_tiks);


  forward_kinematics( &tool_rad2, &th_rad);




  th_tiks = theta_rad_to_tiks( &th_rad);

  theta_print_rad(&th_rad);
  theta_print_tiks(&th_tiks);
  tool_print(&tool_rad);


  th_tiks = theta_rad_to_tiks( &th_rad);

  //  theta_print_rad(&th_rad);
  //  theta_print_tiks(&th_tiks);
  tool_print(&tool_rad2);
  *-/
  
}
*/

void theta_init_start_tiks(float theta[])
{
  theta[0] = 0; //-30000;
  theta[1] = 0; // 49000;
  theta[2] = 0; //0;
  theta[3] = 0; // 50000;
  theta[4] = 0; //-35000;
  theta[5] = 0; //-35000;

}


/*

void tool_init_starting_values(float tool[])
{
  tool[0]     =  20.932410; //20.931822;
  tool[1]     =  23.874095; //23.874062;
  tool[2]     =  23.513540; //23.513273;
  tool[3]     =  -0.204886; //-0.204880;
  tool[4]     =  -1.466077; //-0.366508;
}*/



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


void tool_print(float tool[])
{
  printf("End Effector state\n");
  printf("  x:     %f\n", tool[0]);
  printf("  y:     %f\n", tool[1]);
  printf("  z:     %f\n", tool[2]);
  printf("  beta1: %f° %f rad \n", tool[3], tool[3]/180*M_PI);
  printf("  beta2: %f° %f rad \n", tool[4], tool[4]/180*M_PI);
  
}


void theta_rad_to_tiks(float theta[])
{
  int i=0;
  for(i=0; i<=5; i++)
    theta[i] =sign_switch[i]* ( theta[i]/(2*M_PI)/(i_gear[i]*i_arm[i])*(enc_rev[i]*4))  + zero_offset[i] - home_offset[i] ;
 

}


void theta_tiks_to_rad(float theta[])
{
  int i=0;
  for(i=0; i<=5; i++)
    theta[i] = sign_switch[i]*(theta[i] - zero_offset[i] +  home_offset[i] ) *(2*M_PI)*(i_gear[i]*i_arm[i])/(enc_rev[i]*4);  
 
}



#endif
