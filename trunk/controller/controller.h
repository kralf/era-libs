/*	Header-file for
 *	BlueBotics ERA-5/1 controller
 *
 * 	Ralf Kaestner    ralf.kaestner@gmail.com
 * 	Last change:     9.5.2008
 */

#ifndef _CONTROLLER_H
#define _CONTROLLER_H

#include "motors.h"

/** \file
  * \brief The BlueBotics ERA-5/1 controller
  *
  * The motion controller for the BlueBotics ERA-5/1.
  */

/** \brief Print the current arm configuration
  * \param[in] stream The output stream that will be used for printing the
  *   current arm configuration.
  */
void era_print_configuration(
  FILE* stream);

/** \brief Initialize communication with the arm and perform homing
  * \param[in] dev The character device the arm is attached to.
  * \param[in] homing_mode The motor homing mode to be used. Possible
  *   values are ERA_HOMING_MODE_NONE, ERA_HOMING_MODE_SENSORS, and
  *   ERA_HOMING_MODE_CURRENT.
  */
void era_init(
  const char* dev,
  int homing_mode);

/** \brief Close communication with the arm
  */
void era_close(void);

/** \brief Move the arm to the predefined home configuration
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  */
void era_move_home(
  double velocity,
  int wait);

/** \brief Move the arm to a specified configuration
  * \param[in] target The target arm configuration.
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  */
void era_move(
  const era_arm_configuration_t* target,
  double velocity,
  int wait);

/** \brief Move the tool to a specified configuration
  * \param[in] target The target tool configuration.
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  */
void era_move_tool(
  const era_tool_configuration_t* target,
  double velocity,
  int wait);

/** \brief Move the arm along a given trajectory
  * \param[in] trajectory An array of arm configurations representing
  *   the arm trajectory.
  * \param[in] timestamps An array of relative timestamps associated with
  *   the arm trajectory points [s].
  */
void era_move_trajectory(
  const era_arm_configuration_t* trajectory,
  const double* timestamps);

/** \brief Move the tool along a given trajectory
  * \param[in] trajectory An array of tool configurations representing
  *   the tool trajectory.
  * \param[in] timestamps An array of relative timestamps associated with
  *   the tool trajectory points [s].
  */
void era_move_tool_trajectory(
  const era_tool_configuration_t* trajectory,
  const double* timestamps);

#endif
