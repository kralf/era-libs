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
  */
void era_move_home(void);

/** \brief Move along a given trajectory
  * \param[in] tool_configurations An array of tool configurations
  *   representing the trajectory.
  * \param[in] timestamps An array of timestamps associated with the
  *   trajectory points [s].
  */
void era_move_trajectory(
  era_tool_configuration_t* tool_configurations,
  double* timestamps);

#endif
