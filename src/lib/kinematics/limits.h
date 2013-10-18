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

#include "base/joint.h"

/** \name Error Codes
  * \brief Predefined kinematic limits error codes
  */
//@{
#define ERA_KINEMATICS_LIMITS_ERROR_NONE               0
#define ERA_KINEMATICS_LIMITS_ERROR_EXCEEDED           1
#define ERA_KINEMATICS_LIMITS_ERROR_SINGULARITY        2
//@}

/** \brief Predefined kinematic limits error descriptions
  */
extern const char* era_kinematics_limits_errors[];

/** \brief Structure defining kinematic limits
  */
typedef struct era_kinematics_limits_t {
  era_joint_state_t min;      //!< The lower joint space limit.
  era_joint_state_t max;      //!< The upper joint space limit.

  era_joint_state_t margin;   //!< The joint space safety margin.
} era_kinematics_limits_t, *era_kinematics_limits_p;

/** \brief Initialize the kinematic limits
  * \param[in] limits The kinematic limits to be initialized.
  * \param[in] min The lower joint space limit.
  * \param[in] max The upper joint space limit.
  * \param[in] margin The joint space safety margin.
  */
void era_kinematics_limits_init(
  era_kinematics_limits_p limits,
  era_joint_state_p min,
  era_joint_state_p max,
  era_joint_state_p margin);

/** \brief Test a joint space state against kinematic limits
  * \param[in] limits The kinematic limits the given joint space state
  *   will be tested against.
  * \param[in] state The joint space state that will be tested against 
  *   the specified kinematic limits.
  * \return The resulting error code.
  */
int era_kinematics_limits_test_state(
  era_kinematics_limits_p limits,
  era_joint_state_p state);

/** \brief Test a joint space path against kinematic limits
  * \param[in] limits The kinematic limits the given path will be
  *   tested against.
  * \param[in] path The joint space path that will be tested against the 
  *   specified kinematic limits.
  * \return The number of bad path points.
  */
ssize_t era_kinematics_limits_test_path(
  era_kinematics_limits_p limits,
  era_joint_path_p path);

#endif
