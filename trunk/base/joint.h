/***************************************************************************
 *   Copyright (C) 2008 by Fritz Stoeckli, Ralf Kaestner                   *
 *   stfritz@ethz.ch, ralf.kaestner@gmail.com                              *
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

#ifndef ERA_JOINT_H
#define ERA_JOINT_H

/** \brief ERA joint space configuration
  * Joint space configuration of the BlueBotics ERA-5/1 robot arm.
  */

#include <stdlib.h>
#include <stdio.h>

/** \brief Predefined joint space configuration error codes
  */
#define ERA_JOINT_ERROR_NONE                  0
#define ERA_JOINT_ERROR_FILE_OPEN             1
#define ERA_JOINT_ERROR_FILE_FORMAT           2
#define ERA_JOINT_ERROR_FILE_CREATE           3
#define ERA_JOINT_ERROR_FILE_WRITE            4

/** \brief Predefined joint space configuration error descriptions
  */
extern const char* era_joint_errors[];

/** \brief Structure defining the joint space configuration
  */
typedef struct era_joint_config_t {
  double shoulder_yaw;    //!< The shoulder's yaw angle [rad], denoted theta1.
  double shoulder_roll;   //!< The shoulder's roll angle [rad], denoted theta2.
  double shoulder_pitch;  //!< The shoulder's pitch angle [rad], denoted theta3.

  double ellbow_pitch;    //!< The ellbow's pitch angle [rad], denoted theta4.

  double tool_roll;       //!< The tool's roll angle [rad], denoted theta6.
  double tool_opening;    //!< The tool's opening angle [rad].
} era_joint_config_t, *era_joint_config_p;

/** \brief Structure defining the joint space trajectory
  */
typedef struct era_joint_trajectory_t {
  ssize_t num_points;          //!< The number of trajectory points.

  era_joint_config_p points;   //!< The joint space configuration points.
  double* timestamps;          //!< The associated timestamps in [s].

  ssize_t num_limit_errors;    //!< The number of trajectory limit errors.
  int* limit_errors;           //!< The limit error associated with each point.
} era_joint_trajectory_t, *era_joint_trajectory_p;

/** \brief Initialize a joint space configuration
  * \param[in] config The joint space configuration to be initialized
  *   with zeros.
  */
void era_joint_init_config(
  era_joint_config_p config);

/** \brief Initialize a joint space trajectory
  * \param[in] trajectory The joint space trajectory to be initialized
  *   with zero configurations.
  * \param[in] num_points The number of joint space trajectory points.
  */
void era_joint_init_trajectory(
  era_joint_trajectory_p trajectory,
  ssize_t num_points);

/** \brief Destroy a joint space trajectory
  * \param[in] trajectory The joint space trajectory to be destroyed.
  */
void era_joint_destroy_trajectory(
  era_joint_trajectory_p trajectory);

/** \brief Print a joint space configuration
  * \param[in] stream The output stream that will be used for printing the
  *   joint space configuration.
  * \param[in] config The joint space configuration that will be printed.
  */
void era_joint_print_config(
  FILE* stream,
  era_joint_config_p config);

/** \brief Print a joint space trajectory
  * \param[in] stream The output stream that will be used for printing the
  *   joint space trajectory.
  * \param[in] trajectory The joint space trajectory that will be printed.
  */
void era_joint_print_trajectory(
  FILE* stream,
  era_joint_trajectory_p trajectory);

/** \brief Read joint space trajectory from file
  * \note A trajectory will be allocated and must be destroyed by the caller.
  * \param[in] filename The name of the file containing the joint
  *   space trajectory.
  * \param[out] trajectory The read joint space trajectory.
  * \return The number of joint space trajectory points read from the file
  *   or the negative error code.
  */
int era_joint_read_trajectory(
  const char* filename,
  era_joint_trajectory_p trajectory);

/** \brief Write joint space trajectory to file
  * \param[in] filename The name of the file the joint space trajectory
  *   will be written to.
  * \param[in] trajectory The joint space trajectory to be written.
  * \return The number of joint space trajectory points written to the file
  *   or the negative error code.
  */
int era_joint_write_trajectory(
  const char* filename,
  era_joint_trajectory_p trajectory);

#endif