/*	Header-file for
 *	Interfacing kinematic system model for BlueBotics ERA-5/1
 *      with EPOS control library
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     7.5.2008
 */

#ifndef _MOTORS_H
#define _MOTORS_H

#include <trajectory.h>

/** \file
  * \brief Interfacing kinematic system model with EPOS controller
  *
  * Providing a set of funcions to use EPOS control library with the
  * kinematic system model for BlueBotics ERA-5/1.
  */

/** \brief Operation modes of the motor controller
  */
#define ERA_OPERATION_MODE_POSITION 0
#define ERA_OPERATION_MODE_VELOCITY 1

/** \brief Homing modes of the motor controller
  */
#define ERA_HOMING_MODE_NONE 0
#define ERA_HOMING_MODE_SENSORS 1
#define ERA_HOMING_MODE_CURRENT 2

/** \brief Wait conditions of the motor controller
  */
#define ERA_TARGET_REACHED 0x0408
#define ERA_HOME_ATTAINED 0x1008

/** \brief Structure defining the motor configuration */
typedef struct {
  int shoulder_yaw;    //!< The shoulder's yaw increments [tik].
  int shoulder_roll;   //!< The shoulder's roll increments [tik].
  int shoulder_pitch;  //!< The shoulder's pitch increments [tik].
  int ellbow_pitch;    //!< The ellbow's pitch increments [tik].
  int tool_roll;       //!< The tool's roll increments [tik].
  int tool_opening;    //!< The tool's opening increments [tik].
} era_motor_configuration_t;

/** \brief Structure defining the motor velocity
  * All components are given in [tik/ms].
  */
typedef era_motor_configuration_t era_motor_velocity_t;

/** \brief Constant defining the number of motor tiks per revolution */
const era_motor_configuration_t era_motor_tiks_per_revolution;
/** \brief Constant defining the motor current limit */
const era_motor_configuration_t era_motor_current_limit;

/** \brief Constant defining the motor zero configuration */
const era_motor_configuration_t era_motor_zero;

/** \brief Constant defining the motor home configuration for sensor homing*/
const era_motor_configuration_t era_motor_home_sensors;
/** \brief Constant defining the motor home configuration for current homing*/
const era_motor_configuration_t era_motor_home_current;
/** \brief Constant defining the motor homing method for sensor homing*/
const era_motor_configuration_t era_motor_homing_method_sensors;
/** \brief Constant defining the motor homing method for current homing*/
const era_motor_configuration_t era_motor_homing_method_current;
/** \brief Constant defining the motor home current threshold */
const era_motor_configuration_t era_motor_home_current_threshold;
/** \brief Constant defining the motor homing velocity */
const era_motor_velocity_t era_motor_homing_velocity;

/** \brief Constant defining the signum of the motor configuration */
const era_motor_configuration_t era_motor_signum;
/** \brief Constant defining the motor gear transmission */
const era_arm_configuration_t era_motor_gear_transmission;

/** \brief Static structure holding the motor home configuration */
static era_motor_configuration_t era_motor_home;

/** \brief Print a motor configuration
  * \param[in] stream The output stream that will be used for printing the
  *   motor configuration.
  * \param[in] motor_configuration The motor configuration that will be printed.
  */
void era_print_motor_configuration(
  FILE* stream,
  const era_motor_configuration_t* motor_configuration);

/** \brief Set initial configuration in motor space
  * All motor configuration components will be set to their initial values,
  *   namely 0.
  * \param[in] motor_configuration The configuration that will be
  *   initialized.
  */
void era_motor_configuration_init(
  era_motor_configuration_t* motor_configuration);

/** \brief Convert arm configuration and velocity into motor space
  * \param[in] arm_configuration The arm configuration to be converted
  *   into motor space. Can be null.
  * \param[in] arm_velocity The arm velocity to be converted
  *   into motor space. Can be null.
  * \param[out] motor_configuration The motor configuration that results
  *   from the conversion. Can be null.
  * \param[in] motor_velocity The motor velocity that results
  *   from the conversion. Can be null.
  */
void era_arm_to_motor(
  const era_arm_configuration_t* arm_configuration,
  const era_arm_velocity_t* arm_velocity,
  era_motor_configuration_t* motor_configuration,
  era_motor_velocity_t* motor_velocity);

/** \brief Convert motor configuration and velocity into arm space
  * \param[in] motor_configuration The motor configuration to be converted
  *   into arm space. Can be null.
  * \param[in] motor_velocity The motor velocity to be converted
  *   into arm space. Can be null.
  * \param[out] arm_configuration The arm configuration that results
  *   from the conversion. Can be null.
  * \param[out] arm_velocity The arm velocity that results
  *   from the conversion. Can be null.
  */
void era_motor_to_arm(
  const era_motor_configuration_t* motor_configuration,
  const era_motor_velocity_t* motor_velocity,
  era_arm_configuration_t* arm_configuration,
  era_arm_velocity_t* arm_velocity);

/** \brief Initialize the motor communication
  * Initializes CAN communication to the motors.
  * \param[in] dev The character device the motor controllers are attached to.
  */
void era_motors_init(
  const char* dev);

/** \brief Closes the motor communication
  * Closes CAN communication to the motors.
  */
void era_motors_close(void);

/** \brief Wait for a motor condition
  * \param[in] condition The wait condition. Possible values are
  *   ERA_TARGET_REACHED and ERA_HOME_ATTAINED.
  */
void era_motors_wait(
  int condition);

/** \brief Set the specified motor operation mode
  * \param[in] operation_mode The motor operation mode to be set. Possible
  *   values are ERA_OPERATION_MODE_POSITION and ERA_OPERATION_MODE_VELOCITY.
  * \return 1 on success, 0 otherwise.
  */
int era_motors_set_mode(
  int operation_mode);

/** \brief Home the motors using the specified mode
  * \param[in] homing_mode The motor homing mode to be used. Possible
  *   values are ERA_HOMING_MODE_NONE, ERA_HOMING_MODE_SENSORS, and
  *   ERA_HOMING_MODE_CURRENT.
  */
void era_motors_home(
  int homing_mode);

/** \brief Get arm configuration
  * \param[out] arm_configuration The current arm configuration.
  */
void era_position_mode_get(
  era_arm_configuration_t* arm_configuration);

/** \brief Set arm configuration and velocity
  * \param[in] arm_configuration The arm configuration to be set.
  * \param[in] arm_velocity The arm profile velocity to be set.
  */
void era_position_mode_set(
  const era_arm_configuration_t* arm_configuration,
  const era_arm_velocity_t* arm_velocity);

/** \brief Set motor velocities to 0
  */
void era_velocity_mode_zero(void);

/** \brief Set motor velocities
  * \param[in] arm_velocity The arm profile velocity to be set.
  */
void era_velocity_mode_set(
  const era_arm_velocity_t* arm_velocity);

/** \brief Evaluate the error for a target arm configuration
  * \param[in] arm_configuration The target arm configuration for which
  *   the position error will be evaluated.
  * \return The square root of the squared elements of the configuration error.
  */
double era_position_error(
  const era_arm_configuration_t* arm_configuration);

#endif
