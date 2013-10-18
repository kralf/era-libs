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

#ifndef ERA_MOTORS_VELOCITY_H
#define ERA_MOTORS_VELOCITY_H

/** \brief ERA motor velocity control
  * Motor velocity control implementation for the BlueBotics ERA-5/1.
  */

#include <libepos/velocity.h>

#include "base/velocity.h"

#include "motors/velocity.h"
#include "motors/motors.h"

/** \name Error Codes
  * \brief Predefined motor velocity control error codes
  */
//@{
#define ERA_MOTORS_VELOCITY_ERROR_NONE               0
#define ERA_MOTORS_VELOCITY_ERROR_START              1
#define ERA_MOTORS_VELOCITY_ERROR_SET                2
//@}

/** \brief Predefined motor velocity control error descriptions
  */
extern const char* era_motors_velocity_errors[];

/** \brief Start motor velocity control operation
  * \param[in] motors The opened motors to start the velocity control 
  *   operation for.
  * \return The resulting error code.
  */
int era_motors_velocity_start(
  era_motors_p motors);

/** \brief Stop motor velocity control operation
  * \param[in] motors The opened motors to stop the velocity control 
  *   operation for.
  */
void era_motors_velocity_stop(
  era_motors_p motors);

/** \brief Set motor velocity space state
  * \param[in] motors The opened motors to set the velocity for.
  * \param[in] vel_state The velocity space state to be set.
  * \return The resulting error code.
  */
int era_motors_velocity_set_state(
  era_motors_p motors,
  era_velocity_state_p vel_state);

#endif
