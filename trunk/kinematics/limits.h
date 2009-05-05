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

#ifndef ERA_KINEMATICS_LIMITS_H
#define ERA_KINEMATICS_LIMITS_H

/** \brief ERA kinematic limits
  * Kinematic limits of the BlueBotics ERA-5/1 robot arm.
  */

#include <base/joint.h>

/** \brief Predefined kinematic limits error codes
  */
#define ERA_KINEMATICS_LIMITS_ERROR_NONE               0
#define ERA_KINEMATICS_LIMITS_ERROR_EXCEEDED           1
#define ERA_KINEMATICS_LIMITS_ERROR_SINGULARITY        2

/** \brief Predefined kinematic limits error descriptions
  */
extern const char* era_kinematics_limits_errors[];

/** \brief Structure defining kinematic limits
  */
typedef struct era_kinematics_limits_t {
  era_joint_config_t min;     //!< The lower configuration space limit.
  era_joint_config_t max;     //!< The upper configuration space limit.

  era_joint_config_t margin;  //!< The configuration space safety margin.
} era_kinematics_limits_t, *era_kinematics_limits_p;

/** \brief Initialize the kinematic limits
  * \param[in] limits The kinematic limits to be initialized.
  * \param[in] min The lower configuration space limit.
  * \param[in] max The upper configuration space limit.
  * \param[in] margin The configuration space safety margin.
  */
void era_kinematics_limits_init(
  era_kinematics_limits_p limits,
  era_joint_config_p min,
  era_joint_config_p max,
  era_joint_config_p margin);

/** \brief Test a joint space configuration against kinematic limits
  * \param[in] limits The kinematic limits the given joint space
  *   configuration will be tested against.
  * \param[in] config The joint space configuration that will be tested
  *   against the specified kinematic limits.
  * \return The resulting error code.
  */
int era_kinematics_limits_test_config(
  era_kinematics_limits_p limits,
  era_joint_config_p config);

/** \brief Test a joint space trajectory against kinematic limits
  * \param[in] limits The kinematic limits the given trajectory will be
  *   tested against.
  * \param[in] trajectory The joint space trajectory that will be tested
  *   against the specified kinematic limits.
  * \return The number of bad trajectory points.
  */
ssize_t era_kinematics_limits_test_trajectory(
  era_kinematics_limits_p limits,
  era_joint_trajectory_p trajectory);

#endif
