/*	Header-file for
 *      Trayectory Generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include "kinematics.h"

/** \file
  * \brief Kinematic trajectory generation
  * Monotone Cubic Interpolation is used to create velocity profiles in
  * joint space for given via points in tool space.
  */

/** \brief Structure defining the arm velocity
  * All components are given in [rad/s].
  */
typedef era_arm_configuration_t era_arm_velocity_t;

/** \brief Structure defining trajectory error states
  */
typedef struct {
  int point;              //!< The point index.
  int limits_exceeded;    //!< Indicates configuration space limits exceeded.
  int velocity_exceeded;  //!< Indicates maximum velocity exceeded.
} era_trajectory_error_t;

/** \brief Return the arm velocity limits
  * \param[out] arm_configuration_min The lower limit of the arm
  *   configuration space.
  * \param[out] arm_configuration_max The upper limit of the arm
  *   configuration space.
  */
void era_get_velocity_limits(
  era_arm_velocity_t* arm_velocity_max);

/** \brief Test an arm velocity against velocity limits
  * \param[in] arm_velocity The arm velocity that will be tested
  *   against the velocity limits.
  * \return 1 if the provided arm velocity exceeds velocity limits,
  *   0 otherwise.
  */
int era_test_velocity_limits(
  era_arm_velocity_t* arm_velocity);

/** \brief Read trajectory information from file
  * \param[in] filename The name of the file containing the trajectory.
  * \param[out] tool_configurations An array of tool configurations
  *   representing the trajectory.
  *   The array will be allocated and must be freed by the caller.
  * \param[out] timestamps An array of timestamps associated with the
  *   trajectory points [s].
  *   The array will be allocated and must be freed by the caller.
  * \return The number of tool configurations read from the file.
  */
int era_read_trajectory(
  const char *filename,
  era_tool_configuration_t** tool_configurations,
  double** timestamps);

/** \brief Calculate a trajectory velocity profile
  * \param[in] tool_configurations An array containing the trajectory
  *   points in tool configuration space.
  * \param[in] num_tool_configurations The number of tool configuration
  *   points in the trajectory.
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
int era_trajectory_velocities(
  era_tool_configuration_t* tool_configurations,
  double* timestamps,
  int num_tool_configurations,
  double dt,
  era_arm_velocity_t** arm_velocities,
  era_trajectory_error_t** errors);

/** \brief Precalculate the gradients for Monotone Cubic Interpolation
  *  The gradients of the first and last point will be set to 0
  * \param[in] tool_configurations An array containing the trajectory
  *   points in tool configuration space.
  * \param[in] timestamps An array of timestamps associated with the
  *   trajectory points [s].
  * \param[in] num_tool_configurations The number of tool configuration
  *   points in the trajectory.
  * \param[out] tool_configuration_gradients An array containing the gradients
  *   at each trajectory point in tool configuration space.
  *   The array must be preallocated by the caller.
  */
void era_trajectory_mci_gradients(
  era_tool_configuration_t*  tool_configurations,
  int num_tool_configurations,
  era_tool_configuration_t* tool_configuration_gradients);

/** \brief Perform Monotone Cubic Interpolation on a trajectory
  * \param[in] tool_configurations An array containing the trajectory
  *   points in tool configuration space.
  * \param[in] timestamps An array of timestamps associated with the
  *   trajectory points [s].
  * \param[in] tool_configuration_gradients An array containing the gradients
  *   at each trajectory point in tool configuration space.
  * \param[in] num_tool_configurations The number of tool configuration
  *   points in the trajectory.
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
  era_tool_configuration_t* tool_configurations,
  double* timestamps,
  era_tool_configuration_t* tool_configuration_gradients,
  int num_tool_configurations,
  double dt,
  era_arm_velocity_t** arm_velocities,
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
