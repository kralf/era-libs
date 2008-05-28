/*	Header-file for
 *      Velocity profile generation for BlueBotics ERA-5/1
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     28.5.2008
 */

#ifndef _VELOCITY_PROFILE_H
#define _VELOCITY_PROFILE_H

#include "trajectory.h"
#include "velocity.h"

/** \file
  * \brief Velocity profile generation
  * Generates velocity profiles for given arm trajectories.
  */

/** \brief Print a velocity profile
  * \param[in] stream The output stream that will be used for printing the
  *   velocity profile.
  * \param[in] arm_velocities The arm velocities that will be printed.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the arm velocities [s].
  * \param[in] num_velocities The number of arm velocities in the profile.
  */
void era_print_velocity_profile(
  FILE* stream,
  const era_arm_velocity_t* arm_velocities,
  const double* timestamps,
  int num_configurations);

/** \brief Test a velocity profile against velocity limits
  * \param[in] arm_velocities The arm velocities that will be tested
  *   against the velocity limits. If null, the result will always
  *   be ERA_ERROR_NONE.
  * \param[in] num_velocities The number of arm velocities in the
  *   profile.
  * \return The resulting error code.
  */
int era_test_velocity_profile_limits(
  const era_arm_velocity_t* arm_velocities,
  int num_velocities);

/** \brief Generate a trajectory velocity profile
  * \param[in] trajectory An array containing the trajectory configurations
  *   in arm space.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the trajectory configurations [s].
  * \param[in] num_configurations The number of configurations contained
  *   in the arm trajectory.
  * \param[out] arm_velocities An array of arm velocities representing
  *   the velocity profile.
  */
void era_velocity_profile(
  const era_arm_configuration_t* arm_trajectory,
  const double* timestamps,
  int num_configurations,
  era_arm_velocity_t* arm_velocities);

#endif
