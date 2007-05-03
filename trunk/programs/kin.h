/*	Header-file for 
 *      Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    1.5.2007
 */



#ifndef _KIN_H
#define _KIN_H


/* define data typ for joint angles */
typedef struct {
  float theta1;
  float theta2;
  float theta3;
  float theta4;
  float theta6;
} t_theta;

/* define data typ for cartesian vector */
typedef struct {
  float x;
  float y;
  float z;

} t_cartesian;

/* define data typ for End-Effector state */
typedef struct {
  float x;
  float y;
  float z;
  float beta1;
  float beta2;
} t_target;



/* calculate kinematic model */
void inverse_kinematics(t_target* target,t_theta* theta);
void forward_kinematics(t_target* target,t_theta* theta);




#endif
