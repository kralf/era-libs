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

#ifndef ERA_DYNAMICS_H
#define ERA_DYNAMICS_H

/** \brief ERA dynamic system model
  * Dynamic model for BlueBotics ERA-5/1 robot arm.
  */

#include <base/joint.h>
#include <base/velocity.h>

#include "limits.h"

/** \brief Calculate the velocity space configuration of a linear motion
  * Calculates the velocity space configuration from a defined
  * transition time.
  * \param[in] start_config The joint space start configuration.
  * \param[in] end_config The joint space goal configuration.
  * \param[in] time The time required to move from the joint space start
  *   configuration to the joint space goal configuration in [s].
  * \param[out] vel_config The resulting velocity space configuration.
  */
void era_dynamics_linear_config(
  era_joint_config_p start_config,
  era_joint_config_p end_config,
  double time,
  era_velocity_config_p vel_config);

/** \brief Calculate the velocity space profile of a linear motion
  * Calculates the velocity space profile from a joint space trajectory
  * with defined transition times.
  * \note The joint space trajectory and the velocity space profile should
  *   be equal in the number of points.
  * \param[in] trajectory The joint space trajectory that will be used
  *   to calculate the velocity space profile.
  * \param[out] profile The resulting velocity space profile.
  * \return The number of calculated profile points.
  */
ssize_t era_dynamics_linear_profile(
  era_joint_trajectory_p trajectory,
  era_velocity_profile_p profile);

/** \brief Calculate the velocity space configuration of a linear limit motion
  * Calculates the velocity space configuration from some dynamic limits
  * and a defined velocity factor. The largest velocity space component will
  * be chosen to be the product of the respective velocity limit and the
  * velocity factor.
  * \param[in] start_config The joint space start configuration.
  * \param[in] end_config The joint space goal configuration.
  * \param[in] limits The dynamic limits that will be used to calculate
  *   the velocity space configuration.
  * \param[in] vel_factor A velocity factor in the range 0 to 1.
  * \param[out] vel_config The resulting velocity space configuration.
  * \return The transition time in [s].
  */
double era_dynamics_limit_config(
  era_joint_config_p start_config,
  era_joint_config_p end_config,
  era_dynamics_limits_p limits,
  double vel_factor,
  era_velocity_config_p vel_config);

/** \brief Calculate the velocity space profile of a linear limit motion
  * Calculates the velocity space profile from some dynamic limits
  * and a defined velocity factor. For each trajectory point, the largest
  * velocity space component will be chosen to be the product of the respective
  * velocity limit and the velocity factor.
  * \note The joint space trajectory and the velocity space profile should
  *   be equal in the number of points.
  * \param[in] trajectory The joint space trajectory that will be used
  *   to calculate the velocity space profile.
  * \param[in] limits The dynamic limits that will be used to calculate
  *   the velocity space profile.
  * \param[in] vel_factor A velocity factor in the range 0 to 1.
  * \param[out] profile The resulting velocity space profile.
  * \return The number of calculated profile points.
  */
ssize_t era_dynamics_limit_profile(
  era_joint_trajectory_p trajectory,
  era_dynamics_limits_p limits,
  double vel_factor,
  era_velocity_profile_p profile);

#endif
