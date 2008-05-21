/*	Header-file for
 *      Velocity calculations for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
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
const era_arm_velocity_t era_arm_velocity_max;

/** \brief Print an arm velocity
  * \param[in] stream The output stream that will be used for printing the
  *   arm velocity.
  * \param[in] arm_velocity The arm velocity that will be printed.
  */
void era_print_velocity(
  FILE* stream,
  const era_arm_velocity_t* arm_velocity);

/** \brief Test an arm velocity against velocity limits
  * \param[in] arm_velocity The arm velocity that will be tested
  *   against the velocity limits.
  * \return 1 if the provided arm velocity exceeds velocity limits,
  *   0 otherwise.
  */
int era_test_velocity_limits(
  const era_arm_velocity_t* arm_velocity);

#endif
