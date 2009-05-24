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

#include <motors/motors.h>
#include <security/security.h>
#include <kinematics/limits.h>
#include <dynamics/limits.h>

#include "geometry.h"
#include "tool.h"
#include "joint.h"
#include "velocity.h"
#include "acceleration.h"

/** \brief Predefined ERA constants
  */
#define ERA_PARAMETER_ARM_SECURITY_FUNC             "security-func"
#define ERA_PARAMETER_ARM_ESTOP_CHANNEL             "estop-channel"
#define ERA_PARAMETER_ARM_SWITCH_CHANNEL            "switch-channel"
#define ERA_PARAMETER_ARM_UPPER_LENGTH              "upper-length"
#define ERA_PARAMETER_ARM_LOWER_LENGTH              "lower-length"
#define ERA_PARAMETER_ARM_TOOL_LENGTH               "tool-length"

#define ERA_PARAMETER_JOINT_MIN_POSITION            "min-pos"
#define ERA_PARAMETER_JOINT_MAX_POSITION            "max-pos"
#define ERA_PARAMETER_JOINT_POSITION_MARGIN         "pos-margin"
#define ERA_PARAMETER_JOINT_MAX_VELOCITY            "max-vel"
#define ERA_PARAMETER_JOINT_MAX_ACCELERATION        "max-accel"

/** \brief Predefined ERA error codes
  */
#define ERA_ERROR_NONE                     0
#define ERA_ERROR_OPEN                     1
#define ERA_ERROR_CLOSE                    2
#define ERA_ERROR_HOME                     3
#define ERA_ERROR_MOVE                     4
#define ERA_ERROR_LIMITS                   5

/** \brief Structure defining the BlueBotics ERA-5/1
  */
typedef struct era_arm_t {
  era_motors_t motors;                 //!< The motors of the arm.
  era_security_t security;             //!< The arm's security module.

  era_geometry_t geometry;             //!< The geometry of the arm.
  era_kinematics_limits_t kin_limits;  //!< The arm's kinematic limits.
  era_dynamics_limits_t dyn_limits;    //!< The arm's dynamic limits.

  era_config_t config;                 //!< The arm's configuration parameters.
} era_arm_t, *era_arm_p;

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
  * \note The arm will be moved such that all joints finish their linear
  *    motion simultaneously.
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





/** \brief Set arm configuration and velocity
  * \param[in] configuration The arm configuration to be set. Can be null.
  * \param[in] velocity The arm velocity to be set. Cannot be null.
  * \return The resulting error code.
  */
// int era_set_configuration(
//   const era_arm_configuration_t* configuration,
//   const era_arm_velocity_t* velocity);

/** \brief Read the arm sensors to a callback handler
  * \param[in] handler The callback handler that will receive the sensor
  *   reading updates.
  * \param[in] frequency The sensor data acquisition frequency.
  * \return The resulting error code.
  */
// int era_read(
//   era_read_handler_t handler,
//   double frequency);

/** \brief Move the arm to a specified configuration
  * \param[in] target The target arm configuration.
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  * \return The resulting error code.
  */
// int era_move(
//   const era_arm_configuration_t* target,
//   double velocity,
//   int wait);

/** \brief Move the arm to the predefined home configuration
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  * \return The resulting error code.
  */
// int era_move_home(
//   double velocity,
//   int wait);

/** \brief Move the tool to a specified configuration
  * \param[in] target The target tool configuration.
  * \param[in] velocity The velocity of the arm in the range of 0 to 1.
  * \param[in] wait If 0, return instantly, wait for completion of the move
  *   operation otherwise.
  * \return The resulting error code.
  */
// int era_move_tool(
//   const era_tool_configuration_t* target,
//   double velocity,
//   int wait);

/** \brief Move the arm along a given trajectory
  * Initially, the arm will be positioned in the start configuration. The
  * controller will then execute a velocity profile.
  * \param[in] trajectory An array of arm configurations representing
  *   the arm trajectory.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the arm trajectory points [s].
  * \param[in] num_configurations The number of configurations contained
  *   in the arm trajectory.
  * \return The resulting error code.
  */
// int era_move_trajectory(
//   const era_arm_configuration_t* trajectory,
//   const double* timestamps,
//   int num_configurations);

/** \brief Move the tool along a given trajectory
  * Initially, the arm will be positioned in the start configuration. The
  * controller will then execute a velocity profile.
  * \param[in] trajectory An array of tool configurations representing
  *   the tool trajectory.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the tool trajectory points [s].
  * \param[in] num_configurations The number of configurations contained
  *   in the tool trajectory.
  * \return The resulting error code.
  */
// int era_move_tool_trajectory(
//   const era_tool_configuration_t* trajectory,
//   const double* timestamps,
//   int num_configurations);

/** \brief Execute a given velocity profile
  * Starts the controller on the given velocity profile.
  * \param[in] arm_velocities The arm velocities that will be fed to the
  *   motors.
  * \param[in] timestamps An array of absolute timestamps associated with
  *   the arm velocities [s].
  * \param[in] num_velocities The number of velocities contained in the
  *   profile.
  * \return The resulting error code.
  */
// int era_move_velocity_profile(
//   const era_arm_velocity_t* arm_velocities,
//   const double* timestamps,
//   int num_velocities);

/** \brief Evaluate the error for a specified target arm configuration
  * \param[in] target The target arm configuration for which the configuration
  *   error will be evaluated.
  * \return The square root of the squared elements of the configuration error.
  */
// double era_get_configuration_error(
//   const era_arm_configuration_t* target);

#endif
