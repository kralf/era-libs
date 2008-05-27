/*	Header-file for
 *      File I/O for BlueBotics ERA-5/1
 *
 * 	Ralf Kaestner    ralf.kaestner@gmail.com
 * 	Last change:     27.5.2008
 */

#ifndef _IO_H
#define _IO_H

#include "kinematics.h"
#include "velocity.h"

/** \file
  * \brief Kinematics file I/O
  * File I/O functions for reading and writing trajectories.
  */

/** \brief Read arm trajectory from file
  * \param[in] filename The name of the file containing the arm trajectory.
  * \param[out] arm_trajectory An array of arm configurations representing
  *   the trajectory.
  *   The array will be allocated and must be freed by the caller.
  * \param[out] timestamps An array of absolute timestamps associated with
  *   the trajectory configurations [s].
  *   If the file contains timestamps, the array will be allocated and must
  *   be freed by the caller.
  * \return The number of arm configurations read from the file or the
  *   negative error code.
  */
int era_read_arm_trajectory(
  const char *filename,
  era_arm_configuration_t** arm_trajectory,
  double** timestamps);

/** \brief Write arm trajectory to file
  * \param[in] filename The name of the file the arm trajectory will be
  *   written to.
  * \param[in] arm_trajectory An array of arm configurations representing
  *   the trajectory.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory configurations [s]. Can be null.
  * \param[in] num_configurations The number of configurations contained
  *   in the arm trajectory.
  * \return The number of arm configurations written to the file or the
  *   negative error code.
  */
int era_write_arm_trajectory(
  const char *filename,
  const era_arm_configuration_t* arm_trajectory,
  const double* timestamps,
  int num_configurations);

/** \brief Read tool trajectory from file
  * \param[in] filename The name of the file containing the tool trajectory.
  * \param[out] tool_trajectory An array of tool configurations representing
  *   the trajectory.
  *   The array will be allocated and must be freed by the caller.
  * \param[out] timestamps An array of absolute timestamps associated with
  *   the trajectory configurations [s].
  *   If the file contains timestamps, the array will be allocated and must
  *   be freed by the caller.
  * \return The number of tool configurations read from the file or the
  *   negative error code.
  */
int era_read_tool_trajectory(
  const char *filename,
  era_tool_configuration_t** tool_trajectory,
  double** timestamps);

/** \brief Write tool trajectory to file
  * \param[in] filename The name of the file the tool trajectory will be
  *   written to.
  * \param[in] tool_trajectory An array of tool configurations representing
  *   the trajectory.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory configurations [s]. Can be null.
  * \param[in] num_configurations The number of configurations contained
  *   in the tool trajectory.
  * \return The number of tool configurations written to the file or the
  *   negative error code.
  */
int era_write_tool_trajectory(
  const char *filename,
  const era_tool_configuration_t* tool_trajectory,
  const double* timestamps,
  int num_configurations);

/** \brief Read velocity profile from file
  * \param[in] filename The name of the file containing the velocity profile.
  * \param[out] velocity_profile An array of arm velocities representing
  *   the velocity profile.
  *   The array will be allocated and must be freed by the caller.
  * \param[out] timestamps An array of absolute timestamps associated with
  *   the arm velocities [s].
  *   The array will be allocated and must be freed by the caller.
  * \return The number of arm velocities read from the file or the
  *   negative error code.
  */
int era_read_velocity_profile(
  const char *filename,
  era_arm_velocity_t** velocity_profile,
  double** timestamps);

/** \brief Write velocity profile to file
  * \param[in] filename The name of the file the velocity profile will be
  *   written to.
  * \param[in] velocity_profile An array of arm velocities representing
  *   the trajectory.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the arm velocities [s].
  * \param[in] num_velocities The number of arm velocities contained
  *   in the velocity profile.
  * \return The number of arm velocities written to the file or the
  *   negative error code.
  */
int era_write_velocity_profile(
  const char *filename,
  const era_arm_velocity_t* velocity_profile,
  const double* timestamps,
  int num_velocities);

#endif
