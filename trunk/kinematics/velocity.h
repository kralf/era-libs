/*	Header-file for
 *      Velocity calculations for BlueBotics ERA-5/1
 *
 * 	Ralf Kaestner    ralf.kaestner@gmail.com
 * 	Last change:     6.5.2008
 */

#ifndef _VELOCITY_H
#define _VELOCITY_H

#include "kinematics.h"

/** \file
  * \brief Velocity calculations
  * Definition of velocity limits and velocity calculations.
  */

/** \brief Structure defining the arm velocity
  * All components are given in [rad/s].
  */
typedef era_arm_configuration_t era_arm_velocity_t;

/** \brief Constant defining the upper arm velocity limit */
extern const era_arm_velocity_t era_arm_velocity_max;

/** \brief Initialize an arm velocity
  * \param[in] arm_velocity The arm velocity to be initialized with 0.
  */
void era_init_arm_velocity(
  era_arm_velocity_t* arm_velocity);

/** \brief Print an arm velocity
  * \param[in] stream The output stream that will be used for printing the
  *   arm velocity.
  * \param[in] arm_velocity The arm velocity that will be printed.
  */
void era_print_arm_velocity(
  FILE* stream,
  const era_arm_velocity_t* arm_velocity);

/** \brief Test an arm velocity against velocity limits
  * \param[in] arm_velocity The arm velocity that will be tested
  *   against the velocity limits. If null, the result will
  *   always be ERA_ERROR_NONE.
  * \return The resulting error code.
  */
int era_test_arm_velocity_limits(
  const era_arm_velocity_t* arm_velocity);

/** \brief Calculate an arm velocity for a given configuration change in time
  * \param[in] arm_start_configuration The arm start configuration.
  * \param[in] arm_target_configuration The arm target configuration.
  * \param[in] time The time required to move from the start configuration
  *   to the target configuration.
  * \param[out] arm_velocity The resulting velocity for each component of
  *   the arm configuration space.
  */
void era_arm_velocity(
  const era_arm_configuration_t* arm_start_configuration,
  const era_arm_configuration_t* arm_target_configuration,
  double time,
  era_arm_velocity_t* arm_velocity);

/** \brief Synchronize the joint velocities of the arm
  * Calculates the joint angle velocities such that all joints finish their
  * movements simultaneously. The fastest joint angle velocity will be chosen
  * to be the product of the joint's maximum velocity and the specified
  * velocity.
  * \param[in] arm_start_configuration The arm start configuration.
  * \param[in] arm_target_configuration The arm target configuration.
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[out] arm_velocity The resulting velocity for each component of
  *   the arm configuration space.
  * \return The execution time of the movement [s].
  */
double era_sync_arm_velocity(
  const era_arm_configuration_t* arm_start_configuration,
  const era_arm_configuration_t* arm_target_configuration,
  double velocity,
  era_arm_velocity_t* arm_velocity);

#endif
