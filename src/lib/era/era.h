/***************************************************************************
 *   Copyright (C) 2008 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef ERA_H
#define ERA_H

/** \brief ERA hardware interface
  * Hardware interface functions for the BlueBotics ERA-5/1.
  */

#include "base/arm.h"
#include "base/geometry.h"
#include "base/tool.h"
#include "base/joint.h"
#include "base/velocity.h"
#include "base/acceleration.h"
#include "motors/motors.h"
#include "security/security.h"
#include "kinematics/limits.h"
#include "dynamics/limits.h"

/** \brief Predefined ERA error codes
  */
#define ERA_ERROR_NONE                     0
#define ERA_ERROR_OPEN                     1
#define ERA_ERROR_CLOSE                    2
#define ERA_ERROR_HOME                     3
#define ERA_ERROR_MOVE                     4
#define ERA_ERROR_LIMITS                   5

/** \brief Predefined ERA error descriptions
  */
extern const char* era_errors[];

/** \brief Predefined BlueBotics ERA-5/1 default configuration
  */
extern era_config_t era_default_config;

/** \brief Initialize the arm
  * \param[in] arm The arm to be initialized.
  * \param[in] can_dev The CAN communication device of the arm. If
  *   null, a device will be created from default parameters.
  * \param[in] config The optional ERA configuration parameters. Can be null.
  */
void era_init(
  era_arm_p arm,
  can_device_p can_dev,
  era_config_p config);

/** \brief Initialize the arm from command line arguments
  * \param[in] arm The arm to be initialized.
  * \param[in] argc The number of supplied command line arguments.
  * \param[in] argv The list of supplied command line arguments.
  * \param[in] prefix An optional argument prefix.
  */
void era_init_arg(
  era_arm_p arm,
  int argc,
  char **argv,
  const char* prefix);

/** \brief Destroy the arm
  * \note This method automatically destroys the arm's EPOS nodes.
  * \param[in] arm The arm to be destroyed.
  */
void era_destroy(
  era_arm_p arm);

/** \brief Open the arm
  * \param[in] arm The initialized arm to be opened.
  * \return The resulting error code.
  */
int era_open(
  era_arm_p arm);

/** \brief Close the arm
  * \param[in] arm The opened arm to be closed.
  * \return The resulting error code.
  */
int era_close(
  era_arm_p arm);

/** \brief Print the arm's state
  * \param[in] stream The output stream that will be used for printing the
  *   arm's state.
  * \param[in] arm The opened arm for which the state will be printed.
  * \return The resulting error code.
  */
void era_print_state(
  FILE* stream,
  era_arm_p arm);

/** \brief Retrieve the arm's tool space state
  * \param[in] arm The opened arm to retrieve the tool space state for.
  * \param[out] tool_state The arm's tool space state.
  */
void era_get_tool_state(
  era_arm_p arm,
  era_tool_state_p tool_state);

/** \brief Retrieve the arm's joint space state
  * \param[in] arm The opened arm to retrieve the joint space state for.
  * \param[out] joint_state The arm's joint space state.
  */
void era_get_joint_state(
  era_arm_p arm,
  era_joint_state_p joint_state);

/** \brief Retrieve the arm's velocity space state
  * \param[in] arm The opened arm to retrieve the velocity space state for.
  * \param[out] vel_state The arm's velocity space state.
  */
void era_get_velocity_state(
  era_arm_p arm,
  era_velocity_state_p vel_state);

/** \brief Home the arm
  * \param[in] arm The opened arm to be homed.
  * \return The resulting error code.
  */
int era_home(
  era_arm_p arm);

/** \brief Move the arm to a target joint space state
  * \param[in] arm The opened arm to be moved.
  * \param[in] target_state The target joint space state.
  * \param[in] vel_factor A velocity factor in the range 0 to 1.
  * \return The resulting error code.
  */
int era_move_joints(
  era_arm_p arm,
  era_joint_state_p target_state,
  double vel_factor);

/** \brief Move the arm to a goal tool space state
  * \param[in] arm The opened arm to be moved.
  * \param[in] tool_state The goal tool space state.
  * \param[in] vel_factor A velocity factor in the range 0 to 1.
  * \return The resulting error code.
  */
int era_move_tool(
  era_arm_p arm,
  era_tool_state_p tool_state,
  double vel_factor);

/** \brief Move the arm to its home state
  * \param[in] arm The opened arm to be moved to its home state.
  * \param[in] vel_factor A velocity factor in the range 0 to 1.
  * \return The resulting error code.
  */
int era_move_home(
  era_arm_p arm,
  double vel_factor);

#endif
