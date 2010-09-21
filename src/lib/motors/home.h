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

#ifndef ERA_MOTORS_HOME_H
#define ERA_MOTORS_HOME_H

/** \brief ERA motor homing
  * Motor homing implementation for the BlueBotics ERA-5/1.
  */

#include <libepos/home.h>

#include "motors/home.h"
#include "motors/motors.h"

/** \brief Predefined motor homing error codes
  */
#define ERA_MOTORS_HOME_ERROR_NONE               0
#define ERA_MOTORS_HOME_ERROR_START              1

/** \brief Predefined motor homing error descriptions
  */
extern const char* era_motors_home_errors[];

struct era_security_t;

/** \brief Initialize motor homing operation
  * \param[in] motors The initialized motors to initialize the homing 
  *   operation for.
  */
void era_motors_home_init(
  era_motors_p motors);

/** \brief Start motor homing operation
  * \note This method requires the security module in order to exchange
  *   the motor's input functionalities with respect to their homing direction.
  * \param[in] motors The opened motors to start the homing operation for.
  * \param[in] security The security module of the motors to be homed.
  * \return The resulting error code.
  */
int era_motors_home_start(
  era_motors_p motors,
  struct era_security_t* security);

/** \brief Wait for completion of motor homing
  * \param[in] motors The opened motors to complete the homing operation.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting error code.
  */
int era_motors_home_wait(
  era_motors_p motors,
  double timeout);

/** \brief Stop motor homing operation
  * \param[in] motors The opened motors to stop homing operation for.
  */
void era_motors_home_stop(
  era_motors_p motors);

#endif
