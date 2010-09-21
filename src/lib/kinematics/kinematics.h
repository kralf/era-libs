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

#ifndef ERA_KINEMATICS_H
#define ERA_KINEMATICS_H

/** \brief ERA kinematic system model
  * Inverse and forward kinemtatics for BlueBotics ERA-5/1 robot arm.
  */

#include "base/geometry.h"
#include "base/joint.h"
#include "base/tool.h"

/** \brief Calculate forward kinematics for a single state
  * \param[in] geometry The arm geometry that will be considered in the
  *   forward kinematic calculations.
  * \param[in] joint_state The joint space state for which the tool space 
  *   state will be calculated.
  * \param[out] tool_state The tool space state that results from the 
  *   forward kinematic calculations.
  */
void era_kinematics_forward_state(
  era_geometry_p geometry,
  era_joint_state_p joint_state,
  era_tool_state_p tool_state);

/** \brief Calculate forward kinematics for a path
  * \note The trajectories should be equal in the number of points.
  * \param[in] geometry The arm geometry that will be considered in the
  *   forward kinematic calculations.
  * \param[in] joint_path The joint space path for which the tool space 
  *   path will be calculated.
  * \param[out] tool_path The tool space path that results from the 
  *   forward kinematic calculations.
  * \return The number of converted path points.
  */
ssize_t era_kinematics_forward_path(
  era_geometry_p geometry,
  era_joint_path_p joint_path,
  era_tool_path_p tool_path);

/** \brief Calculate inverse kinematics for a single state
  * \param[in] geometry The arm geometry that will be considered in the
  *   inverse kinematic calculations.
  * \param[in] tool_state The tool space state for which the joint space 
  *   state will be calculated.
  * \param[out] joint_state The joint space state that results from the 
  *   inverse kinematic calculations.
  */
void era_kinematics_inverse_state(
  era_geometry_p geometry,
  era_tool_state_p tool_state,
  era_joint_state_p joint_state);

/** \brief Calculate inverse kinematics for a path
  * \note The trajectories should be equal in the number of points.
  * \param[in] geometry The arm geometry that will be considered in the
  *   inverse kinematic calculations.
  * \param[in] tool_path The tool space path for which the joint space 
  *   path will be calculated.
  * \param[out] joint_path The joint space path that results from
  *   the inverse kinematic calculations.
  * \return The number of converted path points.
  */
ssize_t era_kinematics_inverse_path(
  era_geometry_p geometry,
  era_tool_path_p tool_path,
  era_joint_path_p joint_path);

/** \brief Calculate and set the tool's yaw angle
  * \param[in,out] tool_state The tool space state for which the yaw angle 
  *   will be calculated and set.
  */
void era_kinematics_set_state_yaw(
  era_tool_state_p tool_state);

#endif
