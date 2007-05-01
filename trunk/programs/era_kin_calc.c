/*	Temporary calculations used for
 *      Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change: 1.5.2007
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <getopt.h>
#include <termios.h> 
#include <fcntl.h>   
#include <linux/serial.h>
#include <linux/tty.h>
#include <time.h>
#include <sys/time.h>
#include <sys/types.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <asm/ioctl.h>
#include <unistd.h>
#include <errno.h>



#include <math.h>
#include "era_kin.h"

int main()
{


  long int home_offset[] = {0,50000,50000,50000,50000,180000,30000}; // starting position
  long int zero_offset[] = {0,80000,1000,50000,0,215000,30000};      // down zero position

  float home_zero_offset_deg[] = {0,0,0,0,0,0,0};      
  float home_zero_offset_rad[] = {0,0,0,0,0,0,0};      


  int sign_change[] = {0,1,1,1,1,-1,1};     
  float enc_rev[]={0,500,500,500,500,500,500};
  float i_gear[]={0,0.02,0.02,0.02,0.02,0.005,0.01};
  float i_arm[]={0, 0.108695652, 0.119047619, 0.119047619, 0.129032258, 1, 1};

  //  pos = x/(360)/(i_gear[id]*i_arm[id])*(enc_rev[id]*4);

  int id=1;
  for(id=1;id<7;id++)
    {
      home_zero_offset_deg[id] = (home_offset[id] - zero_offset[id]) * (360)*(i_gear[id]*i_arm[id])/(enc_rev[id]*4);
      home_zero_offset_rad[id] = home_zero_offset_deg[id]/180*M_PI;
      printf("home zero offset %i: %f \n",id,      home_zero_offset_deg[id]);
    }

  

  double  theta_1 = home_zero_offset_rad[1]*sign_change[1];
  double  theta_2 = home_zero_offset_rad[2]*sign_change[2];
  double  theta_3 = home_zero_offset_rad[3]*sign_change[3];
  double  theta_4 = home_zero_offset_rad[4]*sign_change[4];
  double  theta_6 = home_zero_offset_rad[6]*sign_change[6];

  double a_3 = 23.05;
  double a_4 = 22.4;
  double a_5 = 18.8;

  
  
  double x = -a_4*cos(theta_4)*sin(theta_1)*sin(theta_3)-a_4*sin(theta_4)*sin(theta_1)*cos(theta_3)
             +a_3*cos(theta_1)*sin(theta_2)*cos(theta_3)+a_4*cos(theta_4)*cos(theta_1)*sin(theta_2)*cos(theta_3)
             -a_3*sin(theta_1)*sin(theta_3)-a_4*sin(theta_4)*cos(theta_1)*sin(theta_2)*sin(theta_3)-sin(theta_1)*a_5;
  
  double y = cos(theta_1)*a_5+a_3*sin(theta_1)*sin(theta_2)*cos(theta_3)+a_4*sin(theta_4)*cos(theta_1)*cos(theta_3)
             +a_4*cos(theta_4)*cos(theta_1)*sin(theta_3)+a_3*cos(theta_1)*sin(theta_3)
             +a_4*cos(theta_4)*sin(theta_1)*sin(theta_2)*cos(theta_3)-a_4*sin(theta_4)*sin(theta_1)*sin(theta_2)*sin(theta_3);
  
  double z = -cos(theta_2)*cos(theta_3)*cos(theta_4)*a_4+cos(theta_2)*sin(theta_3)*sin(theta_4)*a_4
             -cos(theta_2)*cos(theta_3)*a_3+a_3+a_4+a_5;



  printf("forward  x: %f\n", x);
  printf("forward  y: %f\n", y);
  printf("forward  z: %f\n", z);
  printf("forward  beta1: %f\n", theta_1);
  printf("forward  beta2: %f\n", theta_6-theta_2);


}
