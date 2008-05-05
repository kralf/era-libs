/*	Header-file for
 *      Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:    28.5.2007
 */

#ifndef _KINEMATICS_H
#define _KINEMATICS_H

/** \file
  * \brief Kinematic system model
  * Inverse and forward calculations for BlueBotics ERA-5/1 robot arm.
*/

/** \brief Data type defining a cartesian vector
  */
typedef struct {
  double x;
  double y;
  double z;
} t_cartesian;

/** \brief Calculates the end effector orientation
  * \param[in,out] tool Array contains the end effector
  *   position and orientation coordinates <br>
  *   x[cm], y[cm], z[cm], gamma1[deg] and gamma2[deg] <br>
  *   and the gripper opening
  */
void era_auto_beta1(double tool[]);

/** \brief Forward kinematic calculations
  * \param[out] tool Array contains the end effector
  *   position and orientation coordinates <br>
  *   x[cm], y[cm], z[cm], gamma1[deg] and gamma2[deg] <br>
  *   and the gripper opening
  * \param[in] theta Array contains 5 joint angles
  *   theta1[rad] to theta4[rad] and theta6[rad] <br>
  *   and the gripper opening
  */
void era_forward_kinematics(double tool[], double theta[]);

/** \brief Inverse kinematic calculations
  * \param[in] tool Array contains the end effector
  *   position and orientation coordinates <br>
  *   x[cm], y[cm], z[cm], gamma1[deg] and gamma2[deg] <br>
  *   and the gripper opening
  * \param[out] theta Array contains 5 joint angles
  *   theta1[rad] to theta4[rad] and theta6[rad] <br>
  *   and the gripper opening
  * \return 0, if one of the thetas exceeds its maximal values <br>
  *   1, if all thetas are inside their borders
  */
int era_inverse_kinematics(double tool[], double theta[]);

/** \brief Checks if theta[] is inside the reachable space
  * \param[in] theta Array contains 5 joint angles
  *   theta1[rad] to theta4[rad] and theta6[rad]
  * \return 0, if one of the thetas exceeds its maximal values<br>
  *   1, if all thetas are inside their borders
  */
int era_theta_workspacecheck(double theta[]);

#endif
