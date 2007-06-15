/*	Header-file for 
 *	Connecting
 *      Kinematic system model for BlueBotics ERA-5/1
 *      with libserial
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    28.5.2007
 */


/**     \file
 *      \brief 
 *      Connecting libserial and Kinematics
 *
 *      Providing a set of funcions to use libserial
 *      with the Kinematic system model for BlueBotics ERA-5/1
*/



#ifndef _KIN2EPOS_H
#define _KIN2EPOS_H





//void tool_init_starting_values(double tool[]);

/** Set all theta values to their starting values (zero) */
void theta_init_start_tiks(double theta[] /**< joint angles */
			   );


/** Convertes all input angles from tiks to radians */
void theta_tiks_to_rad(double theta[]   /**< array contains the 5 joint angles <br> 
					 *  theta1 to theta4 and theta6 <br>
					 *  and the gripper opening 
					 */
		       );

/** Convertes all input angles from radians to tiks */
void theta_rad_to_tiks(double theta[]   /**< array contains the 5 joint angles <br> 
					 *  theta1 to theta4 and theta6 <br>
					 *  and the gripper opening 
					 */
		       );


/** Set all motors to Profile Position mode */
void kin2s_position_mode_init();


/** Calculates the joint angle velocities such that all 
 *  joints finish their movements simultaneously using
 *  Profile Position mode. (linear stretch)
 *  The fastest joint angle velocity is set to vel_max.
 */ 
void kin2s_position_mode_calc_vel(double pos[],     /** (input) array with the target
						    *  joint angles and the gripper opening
						    */
				  double pos_old[], /** (input) array with the current
						    *  joint angles and the gripper opening
						    */
				  double vel[],     /** (output) array with the velocities
						    *  joint angles and the gripper opening
						    */
				  double vel_max    /** (input) maximal joint angle velocity[deg/s]
						    */
				  );

/** Set Target Position and Profile Velocity
 *  for using Profile Position mode 
 */
void kin2s_position_mode_set(double pos[],  /**< Target Positions<br>
					    *   array contains the 5 joint angles <br> 
					    *   theta1[rad] to theta4[rad] and theta6[rad] <br>
					    *   and the gripper opening 
					    */
			     double vel[]); /**< Profile Velocities<br>
					    *   array contains the 5 joint angles <br> 
					    *   theta1[rad] to theta4[rad] and theta6[rad] <br>
					    *   and the gripper opening 
					    */

/** \return
 *  the square root of the squared elements of pos_err
 */
double kin2s_position_error(double pos_err[]);



/** Set all motors to Velocity mode */
void kin2s_velocity_mode_init();

/** Set the velocity mode setting values(=demanded velocities) of all motors */
void kin2s_velocity_mode_set(double vel[] /**< array contains the velocity values in [rad/s]*/
			     );

/** Set the velocity mode setting values(=demanded velocities) of all motors to zero*/
void kin2s_velocity_mode_set_zero();



/* Prints the joint angles in degrees  */
void theta_print_rad(double theta[]   /**< array contains the 5 joint angles <br> 
				      *   theta1[rad] to theta4[rad] and theta6[rad] <br>
				      *   and the gripper opening 
				      */
		     );

/* Prints the joint angles in tiks  */
void theta_print_tiks(double theta[]   /**< array contains the 5 joint angles <br> 
				       *   theta1[tiks] to theta4[tiks] and theta6[tiks] <br>
				       *   and the gripper opening 
				       */
		      );

/* Prints the end effector position and orientation */
void tool_print(double tool[]); 


/** Returns the maximum of the two inputs */
double max(double m1, 
	  double m2);


#endif
