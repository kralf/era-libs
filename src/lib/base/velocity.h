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

#ifndef ERA_VELOCITY_H
#define ERA_VELOCITY_H

/** \brief ERA velocity space state
  * Velocity space state of the BlueBotics ERA-5/1 robot arm.
  */

#include <stdlib.h>
#include <stdio.h>

/** \name Error Codes
  * \brief Predefined velocity space state error codes
  */
//@{
#define ERA_VELOCITY_ERROR_NONE                  0
#define ERA_VELOCITY_ERROR_FILE_OPEN             1
#define ERA_VELOCITY_ERROR_FILE_FORMAT           2
#define ERA_VELOCITY_ERROR_FILE_CREATE           3
#define ERA_VELOCITY_ERROR_FILE_WRITE            4
//@}

/** \brief Predefined velocity space state error descriptions
  */
extern const char* era_velocity_errors[];

/** \brief Structure defining the velocity space state
  */
typedef struct era_velocity_state_t {
  double shoulder_yaw;    //!< The shoulder's yaw velocity in [rad/s].
  double shoulder_roll;   //!< The shoulder's roll velocity in [rad/s].
  double shoulder_pitch;  //!< The shoulder's pitch velocity in [rad/s].

  double elbow_pitch;     //!< The elbow's pitch velocity in [rad/s].

  double tool_roll;       //!< The tool's roll velocity in [rad/s].
  double tool_opening;    //!< The tool's opening velocity in [rad/s].
} era_velocity_state_t, *era_velocity_state_p;

/** \brief Structure defining the velocity space profile
  */
typedef struct era_velocity_profile_t {
  ssize_t num_points;           //!< The number of profile points.

  era_velocity_state_p points;  //!< The points of the velocity profile.
  double* timestamps;           //!< The associated timestamps in [s].

  ssize_t num_limit_errors;     //!< The number of profile limit errors.
  int* limit_errors;            //!< The limit error associated with each point.
} era_velocity_profile_t, *era_velocity_profile_p;

/** \brief Initialize a velocity space state
  * \param[in] state The velocity space state to be initialized with zeros.
  */
void era_velocity_init_state(
  era_velocity_state_p state);

/** \brief Initialize a velocity space profile
  * \param[in] profile The velocity space profile to be initialized
  *   with zero states.
  * \param[in] num_points The number of velocity space profile points.
  */
void era_velocity_init_profile(
  era_velocity_profile_p profile,
  ssize_t num_points);

/** \brief Destroy a velocity space profile
  * \param[in] profile The velocity space profile to be destroyed.
  */
void era_velocity_destroy_profile(
  era_velocity_profile_p profile);

/** \brief Print a velocity space state
  * \param[in] stream The output stream that will be used for printing the
  *   velocity space state.
  * \param[in] state The velocity space state that will be printed.
  */
void era_velocity_print_state(
  FILE* stream,
  era_velocity_state_p state);

/** \brief Print a velocity space profile
  * \param[in] stream The output stream that will be used for printing the
  *   velocity space profile.
  * \param[in] profile The velocity space profile that will be printed.
  */
void era_velocity_print_profile(
  FILE* stream,
  era_velocity_profile_p profile);

/** \brief Read velocity space profile from file
  * \note A profile will be allocated and must be destroyed by the caller.
  * \param[in] filename The name of the file containing the velocity
  *   space profile.
  * \param[out] profile The read velocity space profile.
  * \return The number of velocity space profile points read from the file
  *   or the negative error code.
  */
int era_velocity_read_profile(
  const char* filename,
  era_velocity_profile_p profile);

/** \brief Write velocity space profile to file
  * \param[in] filename The name of the file the velocity space profile
  *   will be written to.
  * \param[in] profile The velocity space profile to be written.
  * \return The number of velocity space profile points written to the file
  *   or the negative error code.
  */
int era_velocity_write_profile(
  const char* filename,
  era_velocity_profile_p profile);

#endif
