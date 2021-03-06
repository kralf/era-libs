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

/** \brief ERA tool space state
  * Tool space state of the BlueBotics ERA-5/1 robot arm.
  */

#include <stdlib.h>
#include <stdio.h>

/** \name Error Codes
  * \brief Predefined tool space state error codes
  */
//@{
#define ERA_TOOL_ERROR_NONE                  0
#define ERA_TOOL_ERROR_FILE_OPEN             1
#define ERA_TOOL_ERROR_FILE_FORMAT           2
#define ERA_TOOL_ERROR_FILE_CREATE           3
#define ERA_TOOL_ERROR_FILE_WRITE            4
//@}

/** \brief Predefined tool space state error descriptions
  */
extern const char* era_tool_errors[];

/** \brief Structure defining the tool space state
  */
typedef struct era_tool_state_t {
  double x;        //!< The tool's X-coordinate [m].
  double y;        //!< The tool's Y-coordinate [m].
  double z;        //!< The tool's Z-coordinate [m].

  double yaw;      //!< The tool's yaw angle [rad], denoted beta1.
  double roll;     //!< The tool's roll angle [rad], denoted beta2.
  double opening;  //!< The tool's opening angle [rad].
} era_tool_state_t, *era_tool_state_p;

/** \brief Structure defining the tool space path
  */
typedef struct era_tool_path_t {
  ssize_t num_points;          //!< The number of path points.

  era_tool_state_p points;     //!< The points of the tool space path.
  double* timestamps;          //!< The associated timestamps in [s].
} era_tool_path_t, *era_tool_path_p;

/** \brief Initialize a tool space state
  * \param[in] state The tool space state to be initialized with zeros.
  */
void era_tool_init_state(
  era_tool_state_p state);

/** \brief Initialize a tool space path
  * \param[in] path The tool space path to be initialized with zero states.
  * \param[in] num_points The number of tool space path points.
  */
void era_tool_init_path(
  era_tool_path_p path,
  ssize_t num_points);

/** \brief Destroy a tool space path
  * \param[in] path The tool space path to be destroyed.
  */
void era_tool_destroy_path(
  era_tool_path_p path);

/** \brief Print a tool space state
  * \param[in] stream The output stream that will be used for printing the
  *   tool space state.
  * \param[in] state The tool space state that will be printed.
  */
void era_tool_print_state(
  FILE* stream,
  era_tool_state_p state);

/** \brief Print a tool space path
  * \param[in] stream The output stream that will be used for printing the
  *   tool space path.
  * \param[in] path The tool space path that will be printed.
  */
void era_tool_print_path(
  FILE* stream,
  era_tool_path_p path);

/** \brief Read tool space path from file
  * \note A path will be allocated and must be destroyed by the caller.
  * \param[in] filename The name of the file containing the tool space path.
  * \param[out] path The read tool space path.
  * \return The number of tool space path points read from the file
  *   or the negative error code.
  */
int era_tool_read_path(
  const char* filename,
  era_tool_path_p path);

/** \brief Write tool space path to file
  * \param[in] filename The name of the file the tool space path
  *   will be written to.
  * \param[in] path The tool space path to be written.
  * \return The number of tool space path points written to the file
  *   or the negative error code.
  */
int era_tool_write_path(
  const char* filename,
  era_tool_path_p path);

#endif
