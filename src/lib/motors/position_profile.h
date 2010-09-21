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

#ifndef ERA_MOTORS_POSITION_PROFILE_H
#define ERA_MOTORS_POSITION_PROFILE_H

/** \brief ERA motor position profile control
  * Motor position profile control implementation for the BlueBotics ERA-5/1.
  */

#include <libepos/position_profile.h>

#include "base/joint.h"
#include "base/velocity.h"
#include "base/acceleration.h"

#include "motors/motors.h"

/** \brief Predefined motor position profile control error codes
  */
#define ERA_MOTORS_POSITION_PROFILE_ERROR_NONE               0
#define ERA_MOTORS_POSITION_PROFILE_ERROR_START              1

/** \brief Structure defining an arm position profile control operation
  */
typedef struct era_motors_position_profile_t {
  epos_position_profile_t shoulder_yaw;     //!< The shoulder yaw profile.
  epos_position_profile_t shoulder_roll;    //!< The shoulder roll profile.
  epos_position_profile_t shoulder_pitch;   //!< The shoulder pitch profile.

  epos_position_profile_t elbow_pitch;      //!< The elbow pitch profile.

  epos_position_profile_t tool_roll;        //!< The tool roll profile.
  epos_position_profile_t tool_opening;     //!< The tool opening profile.
} era_motors_position_profile_t, *era_motors_position_profile_p;

/** \brief Predefined motor position profile control error descriptions
  */
extern const char* era_motors_position_profile_errors[];

/** \brief Initialize motor position profile control operation
  * \param[in] profile The arm position profile control operation to be
  *   initialized.
  * \param[in] target_state The target joint space state of the arm.
  * \param[in] vel_state The profile velocity of the arm.
  * \param[in] accel_state The profile acceleration of the arm.
  * \param[in] decel_state The profile deceleration of the arm.
  * \param[in] type The position profile type.
  */
void era_motors_position_profile_init(
  era_motors_position_profile_p profile,
  era_joint_state_p target_state,
  era_velocity_state_p vel_state,
  era_acceleration_state_p accel_state,
  era_acceleration_state_p decel_state,
  epos_profile_type_t type);

/** \brief Start motor position profile control operation
  * \param[in] motors The opened motors to start the position profile 
  *    control operation for.
  * \param[in] profile The arm position profile control operation to be
  *   started.
  * \return The resulting error code.
  */
int era_motors_position_profile_start(
  era_motors_p motors,
  era_motors_position_profile_p profile);

/** \brief Wait for completion of a motor position profile control operation
  * \param[in] motors The opened motors to complete a position profile control 
  *   operation.
  * \param[in] timeout The timeout of the wait operation in [s].
  * \return The resulting device error code.
  */
int era_motors_position_profile_wait(
  era_motors_p motors,
  double timeout);

/** \brief Stop motor position profile control operation
  * \param[in] motors The opened motors to stop the position profile control 
  *   operation for.
  */
void era_motors_position_profile_stop(
  era_motors_p motors);

#endif
