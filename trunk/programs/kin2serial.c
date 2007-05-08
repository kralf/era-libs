/*	Connecting
 *      Kinematic system model for BlueBotics ERA-5/1
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
#include<math.h>


void target_user_read(t_target* target)
{
  t_target target_add;
  while(1)
    {
      printf("Add to Position X [cm]: ");
      if(scanf("%f",&(target_add.x)) == 0) break;
      printf("Add to Position Y [cm]: ");
      if(scanf("%f",&(target_add.y)) == 0) break;
      printf("Add to Position Z [cm]: ");
      if(scanf("%f",&(target_add.z)) == 0) break;
      printf("Add to Angle beta1 [deg]: ");
      if(scanf("%f",&(target_add.beta1)) == 0) break;
      printf("Add to Angle beta2 [deg]: ");
      if(scanf("%f",&(target_add.beta2)) == 0) break;

      target->x += target_add.x;
      target->y += target_add.y;
      target->z += target_add.z;
      target->beta1 += target_add.beta1/180*M_PI; 
      target->beta2 += target_add.beta2/180*M_PI; 

      target_print(target);

    }
		
}

void kin2s_position_mode_set(t_target* target_start, t_target* target_add, int steps)
{


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

 
}

void kin2s_init(t_theta* th, t_target* target)
{


  target_init_starting_values( target );
  inverse_kinematics( target, th);
  target_print( target );
  theta_print_rad( th );
  theta_rad_to_tiks( th );
  theta_print_tiks( th );

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


void theta_init_start_tiks(t_theta* th)
{
  th->theta1 = -30000;
  th->theta2 =  49000;
  th->theta3 =  0;
  th->theta4 =  50000;
  th->theta6 = -35000;
  //  long int home_offset[] = {0,50000,50000,50000,50000,180000,30000}; // starting position
  //  long int zero_offset[] = {0,80000,1000,50000,0,215000,30000};      // down zero position
}



void target_init_starting_values(t_target* target)
{
  target->x     =  20.932410; //20.931822;
  target->y     =  23.874095; //23.874062;
  target->z     =  23.513540; //23.513273;
  target->beta1 =  -0.204886; //-0.204880;
  target->beta2 =  -1.466077; //-0.366508;
}



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
  printf("  beta1: %f° %f rad \n", target->beta1*180/M_PI, target->beta1);
  printf("  beta2: %f° %f rad \n", target->beta2*180/M_PI, target->beta2);
  
}


void theta_rad_to_tiks(t_theta* th)
{
  

  float enc_rev[]={0,500,500,500,500,500,500};
  float i_gear[]={0,0.02,0.02,0.02,0.02,0.005,0.01};
  float i_arm[]={0, 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};
  long int zero_offset[] = {0,80000,1000,50000,0,215000,30000};      // down zero position
  long int home_offset[] = {0,50000,50000,50000,50000,180000,30000}; // starting position

  th->theta1 = th->theta1/(2*M_PI)/(i_gear[1]*i_arm[1])*(enc_rev[1]*4) + zero_offset[1] - home_offset[1];  
  th->theta2 = th->theta2/(2*M_PI)/(i_gear[2]*i_arm[2])*(enc_rev[2]*4) + zero_offset[2] - home_offset[2];  
  th->theta3 = th->theta3/(2*M_PI)/(i_gear[3]*i_arm[3])*(enc_rev[3]*4) + zero_offset[3] - home_offset[3];  
  th->theta4 = th->theta4/(2*M_PI)/(i_gear[4]*i_arm[4])*(enc_rev[4]*4) + zero_offset[4] - home_offset[4];  
  th->theta6 = th->theta6/(2*M_PI)/(i_gear[6]*i_arm[6])*(enc_rev[6]*4) + zero_offset[5] - home_offset[5];  
 

}


void theta_tiks_to_rad(t_theta* th)
{
  
  float enc_rev[]={0,500,500,500,500,500,500};
  float i_gear[]={0,0.02,0.02,0.02,0.02,0.005,0.01};
  float i_arm[]={0, 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};
  long int zero_offset[] = {0,80000,1000,50000,0,215000,30000};      // down zero position
  long int home_offset[] = {0,50000,50000,50000,50000,180000,30000}; // starting position

  th->theta1 = (th->theta1 - zero_offset[1] +  home_offset[1]  ) *(2*M_PI)*(i_gear[1]*i_arm[1])/(enc_rev[1]*4);  
  th->theta2 = (th->theta1 - zero_offset[2] +  home_offset[2]  ) *(2*M_PI)*(i_gear[2]*i_arm[2])/(enc_rev[2]*4);  
  th->theta3 = (th->theta1 - zero_offset[3] +  home_offset[3]  ) *(2*M_PI)*(i_gear[3]*i_arm[3])/(enc_rev[3]*4);  
  th->theta4 = (th->theta1 - zero_offset[4] +  home_offset[4]  ) *(2*M_PI)*(i_gear[4]*i_arm[4])/(enc_rev[4]*4);  
  th->theta6 = (th->theta1 - zero_offset[5] +  home_offset[5]  ) *(2*M_PI)*(i_gear[6]*i_arm[6])/(enc_rev[6]*4);  
 
}



#endif
