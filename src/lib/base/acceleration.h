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

#ifndef ERA_ACCELERATION_H
#define ERA_ACCELERATION_H

/** \brief ERA acceleration space state
  * Acceleration space state of the BlueBotics ERA-5/1 robot arm.
  */

#include <stdlib.h>
#include <stdio.h>

/** \name Error Codes
  * \brief Predefined acceleration space state error codes
  */
//@{
#define ERA_ACCELERATION_ERROR_NONE                  0
//@}

/** \brief Predefined acceleration space state error descriptions
  */
extern const char* era_acceleration_errors[];

/** \brief Structure defining the acceleration space state
  */
typedef struct era_acceleration_state_t {
  double shoulder_yaw;    //!< The shoulder's yaw acceleration in [rad/s^2].
  double shoulder_roll;   //!< The shoulder's roll acceleration in [rad/s^2].
  double shoulder_pitch;  //!< The shoulder's pitch acceleration in [rad/s^2].

  double elbow_pitch;     //!< The elbow's pitch acceleration in [rad/s^2].

  double tool_roll;       //!< The tool's roll acceleration in [rad/s^2].
  double tool_opening;    //!< The tool's opening acceleration in [rad/s^2].
} era_acceleration_state_t, *era_acceleration_state_p;

/** \brief Initialize a acceleration space state
  * \param[in] state The acceleration space state to be initialized with zeros.
  */
void era_acceleration_init_state(
  era_acceleration_state_p state);

/** \brief Print a acceleration space state
  * \param[in] stream The output stream that will be used for printing the
  *   acceleration space state.
  * \param[in] state The acceleration space state that will be printed.
  */
void era_acceleration_print_state(
  FILE* stream,
  era_acceleration_state_p state);

#endif
