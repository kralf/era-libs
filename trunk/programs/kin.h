/*	Header-file for 
 *      Kinematic system model for BlueBotics ERA-5/1
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



#ifndef _KIN_H
#define _KIN_H


/* define data typ for joint angles *-/
typedef struct {
  float theta1;
  float theta2;
  float theta3;
  float theta4;
  float theta6;
} t_theta;
*/

/* define data typ for cartesian vector */
typedef struct {
  float x;
  float y;
  float z;

} t_cartesian;

/* define data typ for End-Effector state *-/
typedef struct {
  float x;
  float y;
  float z;
  float beta1;
  float beta2;
} t_tool; 
*/



/** Forward kinematic calculations */
void forward_kinematics(float tool[],   /**< (Output) array contains the end effector 
					 *   position and orientation coordinates <br>
					 *   x[cm], y[cm], z[cm], gamma1[deg] and gamma2[deg] <br>
					 *   and the gripper opening 
					 */
			float theta[]   /**< (Input) array contains 5 joint angles 
					 *   theta1[rad] to theta4[rad] and theta6[rad] <br>
					 *   and the gripper opening 
					 */
			);


/** Inverse kinematic calculations */
void inverse_kinematics(float tool[],   /**< (Input) array contains the end effector 
					 *   position and orientation coordinates <br>
					 *   x[cm], y[cm], z[cm], gamma1[deg] and gamma2[deg] <br>
					 *   and the gripper opening 
					 */
			float theta[]   /**< (Output) array contains 5 joint angles 
					 *   theta1[rad] to theta4[rad] and theta6[rad] <br>
					 *   and the gripper opening 
					 */
			);

/** Checks if theta[] is inside the reachable space
 * 
 * \return
 * 0 if one of the thetas exceeds its maximal values<br>
 * 1 if all thetas are inside their borders
 */
int theta_workspacecheck(float theta[]/**< (Input) array contains 5 joint angles 
				       *   theta1[rad] to theta4[rad] and theta6[rad]
				       */
			 );



#endif
