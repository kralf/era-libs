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

#ifndef ERA_DYNAMICS_LIMITS_H
#define ERA_DYNAMICS_LIMITS_H

/** \brief ERA dynamic limits
  * Dynamic limits of the BlueBotics ERA-5/1 robot arm.
  */

#include "base/velocity.h"
#include "base/acceleration.h"

/** \brief Predefined dynamic limits error codes
  */
#define ERA_DYNAMICS_LIMITS_ERROR_NONE               0
#define ERA_DYNAMICS_LIMITS_ERROR_EXCEEDED           1

/** \brief Predefined dynamic limits error descriptions
  */
extern const char* era_dynamics_limits_errors[];

/** \brief Structure defining dynamic limits
  */
typedef struct era_dynamics_limits_t {
  era_velocity_state_t max_vel;        //!< The upper velocity space limit.

  era_acceleration_state_t min_accel;  //!< The lower acceleration space limit.
  era_acceleration_state_t max_accel;  //!< The upper acceleration space limit.
} era_dynamics_limits_t, *era_dynamics_limits_p;

/** \brief Initialize the dynamic limits
  * \param[in] limits The dynamic limits to be initialized.
  * \param[in] max_vel The upper velocity space limit.
  * \param[in] min_accel The lower acceleration space limit.
  * \param[in] max_accel The upper acceleration space limit.
  */
void era_dynamics_limits_init(
  era_dynamics_limits_p limits,
  era_velocity_state_p max_vel,
  era_acceleration_state_p min_accel,
  era_acceleration_state_p max_accel);

/** \brief Test a velocity space state against dynamic limits
  * \param[in] limits The dynamic limits the given velocity space state 
  *   will be tested against.
  * \param[in] vel_state The velocity space state that will be tested against 
  *   the specified dynamic limits.
  * \return The resulting error code.
  */
int era_dynamics_limits_test_velocity_state(
  era_dynamics_limits_p limits,
  era_velocity_state_p vel_state);

/** \brief Test an acceleration space state against dynamic limits
  * \param[in] limits The dynamic limits the given acceleration space state 
  *   will be tested against.
  * \param[in] accel_state The acceleration space state that will be tested 
  *   against the specified dynamic limits.
  * \return The resulting error code.
  */
int era_dynamics_limits_test_acceleration_state(
  era_dynamics_limits_p limits,
  era_acceleration_state_p accel_state);

/** \brief Test a velocity space profile against dynamic limits
  * \param[in] limits The dynamic limits the given profile will be
  *   tested against.
  * \param[in] profile The velocity space profile that will be tested
  *   against the specified dynamic limits.
  * \return The number of bad profile points.
  */
ssize_t era_dynamics_limits_test_velocity_profile(
  era_dynamics_limits_p limits,
  era_velocity_profile_p profile);

#endif
