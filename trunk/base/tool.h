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

#ifndef ERA_TOOL_H
#define ERA_TOOL_H

/** \brief ERA tool space configuration
  * Tool space configuration of the BlueBotics ERA-5/1 robot arm.
  */

#include <stdlib.h>
#include <stdio.h>

/** \brief Predefined tool space configuration error codes
  */
#define ERA_TOOL_ERROR_NONE                  0
#define ERA_TOOL_ERROR_FILE_OPEN             1
#define ERA_TOOL_ERROR_FILE_FORMAT           2
#define ERA_TOOL_ERROR_FILE_CREATE           3
#define ERA_TOOL_ERROR_FILE_WRITE            4

/** \brief Predefined tool space configuration error descriptions
  */
extern const char* era_tool_errors[];

/** \brief Structure defining the tool space configuration
  */
typedef struct era_tool_config_t {
  double x;        //!< The tool's X-coordinate [m].
  double y;        //!< The tool's Y-coordinate [m].
  double z;        //!< The tool's Z-coordinate [m].

  double yaw;      //!< The tool's yaw angle [rad], denoted beta1.
  double roll;     //!< The tool's roll angle [rad], denoted beta2.
  double opening;  //!< The tool's opening angle [rad].
} era_tool_config_t, *era_tool_config_p;

/** \brief Structure defining the tool space trajectory
  */
typedef struct era_tool_trajectory_t {
  ssize_t num_points;          //!< The number of trajectory points.

  era_tool_config_p points;   //!< The tool space configuration points.
  double* timestamps;          //!< The associated timestamps in [s].
} era_tool_trajectory_t, *era_tool_trajectory_p;

/** \brief Initialize a tool space configuration
  * \param[in] config The tool space configuration to be initialized
  *   with zeros.
  */
void era_tool_init_config(
  era_tool_config_p config);

/** \brief Initialize a tool space trajectory
  * \param[in] trajectory The tool space trajectory to be initialized
  *   with zero configurations.
  * \param[in] num_points The number of tool space trajectory points.
  */
void era_tool_init_trajectory(
  era_tool_trajectory_p trajectory,
  ssize_t num_points);

/** \brief Destroy a tool space trajectory
  * \param[in] trajectory The tool space trajectory to be destroyed.
  */
void era_tool_destroy_trajectory(
  era_tool_trajectory_p trajectory);

/** \brief Print a tool space configuration
  * \param[in] stream The output stream that will be used for printing the
  *   tool space configuration.
  * \param[in] config The tool space configuration that will be printed.
  */
void era_tool_print_config(
  FILE* stream,
  era_tool_config_p config);

/** \brief Print a tool space trajectory
  * \param[in] stream The output stream that will be used for printing the
  *   tool space trajectory.
  * \param[in] trajectory The tool space trajectory that will be printed.
  */
void era_tool_print_trajectory(
  FILE* stream,
  era_tool_trajectory_p trajectory);

/** \brief Read tool space trajectory from file
  * \note A trajectory will be allocated and must be destroyed by the caller.
  * \param[in] filename The name of the file containing the tool
  *   space trajectory.
  * \param[out] trajectory The read tool space trajectory.
  * \return The number of tool space trajectory points read from the file
  *   or the negative error code.
  */
int era_tool_read_trajectory(
  const char* filename,
  era_tool_trajectory_p trajectory);

/** \brief Write tool space trajectory to file
  * \param[in] filename The name of the file the tool space trajectory
  *   will be written to.
  * \param[in] trajectory The tool space trajectory to be written.
  * \return The number of tool space trajectory points written to the file
  *   or the negative error code.
  */
int era_tool_write_trajectory(
  const char* filename,
  era_tool_trajectory_p trajectory);

#endif
