/*	Header-file for
 *      Trayectory generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include "kinematics.h"
#include "velocity.h"

/** \file
  * \brief Kinematic trajectory generation
  * Monotone Cubic Interpolation is used to create velocity profiles in
  * joint space for given via points in tool space.
  */

/** \brief Structure defining trajectory error states */
typedef struct {
  int limits_exceeded;    //!< Indicates configuration space limits exceeded.
  int velocity_exceeded;  //!< Indicates maximum velocity exceeded.
} era_trajectory_error_t;

/** \brief Print a tool trajectory
  * \param[in] stream The output stream that will be used for printing the
  *   tool trajectory.
  * \param[in] tool_trajectory The tool trajectory that will be printed.
  * \param[in] num_configurations The number of tool configurations
  *   in the trajectory.
  */
void era_print_tool_trajectory(
  FILE* stream,
  const era_tool_configuration_t* tool_trajectory,
  int num_configurations);

/** \brief Print an arm trajectory
  * \param[in] stream The output stream that will be used for printing the
  *   arm trajectory.
  * \param[in] arm_trajectory The arm trajectory that will be printed.
  * \param[in] num_configurations The number of arm configurations
  *   in the trajectory.
  */
void era_print_arm_trajectory(
  FILE* stream,
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations);

/** \brief Test an arm trajectory against configuration space limits
  * \param[in] arm_trajectory The arm trajectory that will be tested
  *   against the configuration space limits. If null, the result will
  *   always be ERA_ERROR_NONE.
  * \param[in] num_configurations The number of arm configurations
  *   in the trajectory.
  * \return The resulting error code.
  */
int era_trajectory_test_arm_configuration_limits(
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations);

/** \brief Trajectory forward kinematic calculations
  * \param[in] arm_trajectory The arm trajectory for which the tool
  *   trajectory will be calculated.
  * \param[in] num_configurations The number of arm configurations
  *   in the trajectory.
  * \param[out] tool_trajectory The tool trajectory that results from
  *   the forward kinematic calculations.
  */
void era_trajectory_forward_kinematics(
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations,
  era_tool_configuration_t* tool_trajectory);

/** \brief Trajectory inverse kinematic calculations
  * \param[in] tool_trajectory The tool trajectory for which the arm
  *   trajectory will be calculated.
  * \param[in] num_configurations The number of tool configurations
  *   in the trajectory.
  * \param[out] arm_trajectory The arm trajectory that results from
  *   the inverse kinematic calculations.
  */
void era_trajectory_inverse_kinematics(
  const era_tool_configuration_t* tool_trajectory,
  int num_configurations,
  era_arm_configuration_t* arm_trajectory);

/** \brief Calculate a trajectory velocity profile
  * \param[in] trajectory An array containing the trajectory configurations
  *   in tool space.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s].
  * \param[in] num_configurations The number of tool configurations
  *   in the trajectory.
  * \param[in] dt Duration of a time interval in the velocity profile [s]
  * \param[out] arm_velocities An array containing the velocity profile for
  *   each component of the arm configuration space,
  *   i.e. arm_velocities[i][j] contains the velocity of joint j at time i*dt.
  *   The array will be allocated and must be freed by the caller.
  * \param[out] errors An array containing the error states for each
  *   time interval in the velocity profile.
  *   The array will be allocated and must be freed by the caller.
  * \return The number of time intervals the velocity profile consists of.
  */
int era_trajectory_velocity_profile(
  const era_tool_configuration_t* trajectory,
  const double* timestamps,
  int num_configurations,
  double dt,
  era_arm_velocity_t** velocity_profile,
  era_trajectory_error_t** errors);

/** \brief Precalculate the gradients for Monotone Cubic Interpolation
  *  The gradients of the first and last point will be set to 0
  * \param[in] trajectory An array containing the trajectory configurations
  *   in tool space.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s].
  * \param[in] num_configurations The number of tool configurations
  *   in the trajectory.
  * \param[out] trajectory_gradients An array containing the gradients
  *   at each trajectory configuration in tool space.
  *   The array must be preallocated by the caller.
  */
void era_trajectory_mci_gradients(
  const era_tool_configuration_t* trajectory,
  int num_configurations,
  era_tool_configuration_t* trajectory_gradients);

/** \brief Perform Monotone Cubic Interpolation on a trajectory
  * \param[in] trajectory An array containing the trajectory configurations
  *   in tool space.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s].
  * \param[in] trajectory_gradients An array containing the gradients
  *   at each trajectory configuration in tool space.
  * \param[in] num_configurations The number of tool configurations
  *   in the trajectory.
  * \param[in] dt Duration of a time interval in the velocity profile [s]
  * \param[out] arm_velocities An array containing the velocity profile for
  *   each component of the arm configuration space,
  *   i.e. arm_velocities[i][j] contains the velocity of joint j at time i*dt.
  *   The array will be allocated and must be freed by the caller.
  * \param[out] errors An array containing the error states for each
  *   time interval in the velocity profile.
  *   The array will be allocated and must be freed by the caller.
  * \return The number of time intervals the velocity profile consists of.
  */
int era_trajectory_mci(
  const era_tool_configuration_t* trajectory,
  const double* timestamps,
  const era_tool_configuration_t* trajectory_gradients,
  int num_configurations,
  double dt,
  era_arm_velocity_t** velocity_profile,
  era_trajectory_error_t** errors);

/** \brief Evaluate a cubic trajectory polynomial
  * \param[in] p_a The position at t = 0.
  * \param[in] p_b The position at t = 1.
  * \param[in] m_a The gradient at t = 0.
  * \param[in] m_b The gradient at t = 1.
  * \param[in] t Evaluation takes place at t = [0, 1].
  * \return The position at t.
  */
double era_trajectory_eval(
  double p_a,
  double p_b,
  double m_a,
  double m_b,
  double t);

#endif
