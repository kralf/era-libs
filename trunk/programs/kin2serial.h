/*	Header-file for 
 *	Connecting
 *      Kinematic system model for BlueBotics ERA-5/1
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


#include"kin.h"




/** Returns the maximum of the two inputs */
float max(float m1, 
	  float m2);


void tool_init_starting_values(float tool[]);


void theta_init_start_tiks(float theta[]);

/** Convertes all input angles from tiks to radians */
void theta_tiks_to_rad(float theta[]   /**< array contains the 5 joint angles <br> 
					 *  theta1 to theta4 and theta6 <br>
					 *  and the gripper opening 
					 */
		       );

/** Convertes all input angles from radians to tiks */
void theta_rad_to_tiks(float theta[]   /**< array contains the 5 joint angles <br> 
					 *  theta1 to theta4 and theta6 <br>
					 *  and the gripper opening 
					 */
		       );



/* void kin2s_init(float theta[], float tool[]); */





/** Set all motors to Profile Position mode */
void kin2s_position_mode_init();


/** Calculates the joint angle velocities such that all 
 *  joints finish their movements simultaneously using
 *  Profile Position mode. (linear stretch)
 *  The fastest joint angle velocity is set to vel_max.
 */ 
void kin2s_position_mode_calc_vel(float pos[],     /** (input) array with the target
						    *  joint angles and the gripper opening
						    */
				  float pos_old[], /** (input) array with the current
						    *  joint angles and the gripper opening
						    */
				  float vel[],     /** (output) array with the velocities
						    *  joint angles and the gripper opening
						    */
				  float vel_max    /** (input) maximal joint angle velocity[deg/s]
						    */
				  );

/** Set Target Position and Profile Velocity
 *  for using Profile Position mode 
 */
void kin2s_position_mode_set(float pos[],  /**< Target Positions<br>
					    *   array contains the 5 joint angles <br> 
					    *   theta1[rad] to theta4[rad] and theta6[rad] <br>
					    *   and the gripper opening 
					    */
			     float vel[]); /**  Profile Velocities<br>
					    *   array contains the 5 joint angles <br> 
					    *   theta1[rad] to theta4[rad] and theta6[rad] <br>
					    *   and the gripper opening 
					    */

float kin2s_position_error(float pos[]);



/* Prints the joint angles in degrees  */
void theta_print_rad(float theta[]   /**< array contains the 5 joint angles <br> 
				      *   theta1[rad] to theta4[rad] and theta6[rad] <br>
				      *   and the gripper opening 
				      */
		     );

/* Prints the joint angles in tiks  */
void theta_print_tiks(float theta[]   /**< array contains the 5 joint angles <br> 
				       *   theta1[tiks] to theta4[tiks] and theta6[tiks] <br>
				       *   and the gripper opening 
				       */
		      );

/* Prints the end effector position and orientation */
void tool_print(float tool[]); 
