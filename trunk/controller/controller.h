/*	Header-file for
 *	BlueBotics ERA-5/1 controller
 *
 * 	Ralf Kaestner    ralf.kaestner@gmail.com
 * 	Last change:     9.5.2008
 */

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "kinematics.h"
#include "velocity.h"
#include "trajectory.h"

/** \file
  * \brief The BlueBotics ERA-5/1 controller
  *
  * The motion controller for the BlueBotics ERA-5/1.
  */

/** \brief Constant defining the homing velocity */
extern const era_arm_velocity_t era_homing_velocity;

/** \brief Structure holding the home configuration */
extern era_arm_configuration_t era_home;

/** \brief Print the current arm and tool configuration
  * \param[in] stream The output stream that will be used for printing the
  *   current arm and tool configuration.
  */
void era_print_configuration(
  FILE* stream);

/** \brief Initialize communication with the arm and perform homing
  * \param[in] dev The character device the arm is attached to.
  * \return The resulting error code.
  */
int era_init(
  const char* dev);

/** \brief Close communication with the arm
  */
void era_close(void);

/** \brief Get the current arm configuration and velocity
  * \param[out] configuration The current arm configuration. Can be null.
  * \param[out] velocity The current arm velocity. Can be null.
  */
void era_get_configuration(
  era_arm_configuration_t* configuration,
  era_arm_velocity_t* velocity);

/** \brief Set arm configuration and velocity
  * \param[in] configuration The arm configuration to be set. Can be null.
  * \param[in] velocity The arm velocity to be set. Cannot be null.
  * \return The resulting error code.
  */
int era_set_configuration(
  const era_arm_configuration_t* configuration,
  const era_arm_velocity_t* velocity);

/** \brief Move the arm to a specified configuration
  * \param[in] target The target arm configuration.
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  * \return The resulting error code.
  */
int era_move(
  const era_arm_configuration_t* target,
  double velocity,
  int wait);

/** \brief Move the arm to the predefined home configuration
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  * \return The resulting error code.
  */
int era_move_home(
  double velocity,
  int wait);

/** \brief Move the tool to a specified configuration
  * \param[in] target The target tool configuration.
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  * \return The resulting error code.
  */
int era_move_tool(
  const era_tool_configuration_t* target,
  double velocity,
  int wait);

/** \brief Move the arm along a given trajectory
  * \param[in] trajectory An array of arm configurations representing
  *   the arm trajectory.
  * \param[in] timestamps An array of relative timestamps associated with
  *   the arm trajectory points [s].
  * \return The resulting error code.
  */
int era_move_trajectory(
  const era_arm_configuration_t* trajectory,
  const double* timestamps);

/** \brief Move the tool along a given trajectory
  * \param[in] trajectory An array of tool configurations representing
  *   the tool trajectory.
  * \param[in] timestamps An array of relative timestamps associated with
  *   the tool trajectory points [s].
  * \return The resulting error code.
  */
int era_move_tool_trajectory(
  const era_tool_configuration_t* trajectory,
  const double* timestamps);

/** \brief Evaluate the error for a specified target arm configuration
  * \param[in] target The target arm configuration for which the configuration
  *   error will be evaluated.
  * \return The square root of the squared elements of the configuration error.
  */
double era_get_configuration_error(
  const era_arm_configuration_t* target);

#endif
