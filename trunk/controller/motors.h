/*	Header-file for
 *	Interfacing kinematic system model for BlueBotics ERA-5/1
 *      with EPOS control library
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     7.5.2008
 */

#ifndef _MOTORS_H
#define _MOTORS_H

#include <kinematics.h>

/** \file
  * \brief Interfacing kinematic system model with EPOS controller
  *
  * Providing a set of funcions to use EPOS control library with the
  * kinematic system model for BlueBotics ERA-5/1.
  */

/** \brief Operation modes of the motor controller
  */
#define ERA_MODE_POSITION 0
#define ERA_MODE_VELOCITY 1

/** \brief Structure defining the motor configuration */
typedef struct {
  int shoulder_yaw;    //!< The shoulder's yaw increments [tik].
  int shoulder_roll;   //!< The shoulder's roll increments [tik].
  int shoulder_pitch;  //!< The shoulder's pitch increments [tik].
  int ellbow_pitch;    //!< The ellbow's pitch increments [tik].
  int tool_roll;       //!< The tool's roll increments [tik].
  int tool_opening;    //!< The tool's opening increments [tik].
} era_motor_configuration_t;

/** \brief Print a motor configuration
  * \param[in] stream The output stream that will be used for printing the
  *   motor configuration.
  * \param[in] motor_configuration The motor configuration that will be printed.
  */
void era_print_motor_configuration(
  FILE* stream,
  era_motor_configuration_t* motor_configuration);

/** \brief Set initial configuration in motor space
  * All motor configuration components will be set to their initial values,
  *   namely 0.
  * \param[in] motor_configuration The configuration that will be
  *   initialized.
  */
void era_motor_configuration_init(
  era_motor_configuration_t* motor_configuration);

/** \brief Convert an arm configuration to a motor configuration
  * \param[in] arm_configuration The arm configuration to be converted
  *   into motor space.
  * \param[out] motor_configuration The motor configuration that results
  *   from the conversion.
  */
void era_arm_to_motor(
  era_arm_configuration_t* arm_configuration
  era_motor_configuration_t* motor_configuration);

/** \brief Convert a motor configuration to an arm configuration
  * \param[in] motor_configuration The motor configuration to be converted
  *   into arm space .
  * \param[out] arm_configuration The arm configuration that results
  *   from the conversion.
  */
void era_motor_to_arm(
  era_motor_configuration_t* motor_configuration
  era_arm_configuration_t* arm_configuration);

/** \brief Initialize the motors in the specified mode
  * \param[in] mode The motor mode to be set. Possible values are
  *   ERA_MODE_POSITION and ERA_MODE_VELOCITY.
  */
void era_motors_init(
  int mode);

/** \brief Calculate velocities for position mode operations
  * Calculates the joint angle velocities such that all joints finish their
  * movements simultaneously in profile position mode. The method applies
  * linear stretching. The fastest joint angle velocity is set to velocity_max.
  * \param[in] arm_current_configuration The current arm configuration.
  * \param[in] arm_target_configuration The target arm configuration.
  * \param[out] arm_velocity The resulting velocity for each component of
  *   the arm configuration space.
  * \param[in] velocity_max The maximum joint angle velocity [rad/s].
  */
void era_position_mode_velocities(
  era_arm_configuration_t* arm_current_configuration,
  era_arm_configuration_t* arm_target_configuration,
  era_arm_velocity_t* arm_velocity,
  double velocity_max);

/** \brief Set motor positions and velocities
  * \param[in] arm_configuration The arm configuration to be set.
  * \param[in] arm_velocity The arm profile velocity to be set.
  */
void era_position_mode_set(
  era_arm_configuration_t* arm_configuration,
  era_arm_velocity_t* arm_velocity
);

/** \brief Set motor velocities to 0
  */
void era_velocity_mode_zero(void);

/** \brief Set motor velocities
  * \param[in] arm_velocity The arm profile velocity to be set.
  */
void era_velocity_mode_set(
  era_arm_velocity_t* arm_velocity);

/** \brief Evaluate the position error for a target arm configuration
  * \param[in] arm_configuration The target arm configuration for which
  *   the position error will be evaluated.
  * \return The square root of the squared elements of the position error.
  */
double era_position_error(
  era_arm_configuration_t* arm_configuration);

#endif
