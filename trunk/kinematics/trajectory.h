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
  int point;              //!< The point index.
  int limits_exceeded;    //!< Indicates configuration space limits exceeded.
  int velocity_exceeded;  //!< Indicates maximum velocity exceeded.
} era_trajectory_error_t;

/** \brief Read arm trajectory information from file
  * \param[in] filename The name of the file containing the arm trajectory.
  * \param[out] arm_trajectory An array of arm configurations representing
  *   the trajectory.
  *   The array will be allocated and must be freed by the caller.
  * \param[out] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s].
  *   If the file contains timestamps, the array will be allocated and must
  *   be freed by the caller.
  * \return The number of arm configurations read from the file.
  */
int era_read_arm_trajectory(
  const char *filename,
  era_arm_configuration_t** arm_trajectory,
  double** timestamps);

/** \brief Write arm trajectory information to file
  * \param[in] filename The name of the file the arm trajectory will be
  *   written to.
  * \param[in] arm_trajectory An array of arm configurations representing
  *   the trajectory.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s]. Can be null.
  * \return The number of arm configurations written to the file.
  */
int era_write_arm_trajectory(
  const char *filename,
  const era_arm_configuration_t* arm_trajectory,
  const double* timestamps);

/** \brief Read tool trajectory information from file
  * \param[in] filename The name of the file containing the tool trajectory.
  * \param[out] tool_trajectory An array of tool configurations representing
  *   the trajectory.
  *   The array will be allocated and must be freed by the caller.
  * \param[out] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s].
  *   If the file contains timestamps, the array will be allocated and must
  *   be freed by the caller.
  * \return The number of tool configurations read from the file.
  */
int era_read_tool_trajectory(
  const char *filename,
  era_tool_configuration_t** tool_trajectory,
  double** timestamps);

/** \brief Write tool trajectory information to file
  * \param[in] filename The name of the file the tool trajectory will be
  *   written to.
  * \param[in] tool_trajectory An array of tool configurations representing
  *   the trajectory.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s]. Can be null.
  * \return The number of tool configurations written to the file.
  */
int era_write_tool_trajectory(
  const char *filename,
  const era_tool_configuration_t* tool_trajectory,
  const double* timestamps);

/** \brief Calculate a trajectory velocity profile
  * \param[in] tool_configurations An array containing the trajectory
  *   points in tool configuration space.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s].
=  * \param[in] num_tool_configurations The number of tool configuration
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
  const era_tool_configuration_t* tool_configurations,
  const double* timestamps,
  int num_tool_configurations,
  double dt,
  era_arm_velocity_t** arm_velocities,
  era_trajectory_error_t** errors);

/** \brief Precalculate the gradients for Monotone Cubic Interpolation
  *  The gradients of the first and last point will be set to 0
  * \param[in] tool_configurations An array containing the trajectory
  *   points in tool configuration space.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s].
  * \param[in] num_tool_configurations The number of tool configuration
  *   points in the trajectory.
  * \param[out] tool_configuration_gradients An array containing the gradients
  *   at each trajectory point in tool configuration space.
  *   The array must be preallocated by the caller.
  */
void era_trajectory_mci_gradients(
  const era_tool_configuration_t* tool_configurations,
  int num_tool_configurations,
  era_tool_configuration_t* tool_configuration_gradients);

/** \brief Perform Monotone Cubic Interpolation on a trajectory
  * \param[in] tool_configurations An array containing the trajectory
  *   points in tool configuration space.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory points [s].
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
  const era_tool_configuration_t* tool_configurations,
  const double* timestamps,
  const era_tool_configuration_t* tool_configuration_gradients,
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
