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

#include <base/geometry.h>
#include <base/joint.h>
#include <base/tool.h>

/** \brief Calculate forward kinematics for a single configuration
  * \param[in] geometry The arm geometry that will be considered in the
  *   forward kinematic calculations.
  * \param[in] joint_config The joint space configuration for which the tool
  *   space configuration will be calculated.
  * \param[out] tool_config The tool space configuration that results from
  *   the forward kinematic calculations.
  */
void era_kinematics_forward_config(
  era_geometry_p geometry,
  era_joint_config_p joint_config,
  era_tool_config_p tool_config);

/** \brief Calculate forward kinematics for a trajectory
  * \note The trajectories should be equal in the number of points.
  * \param[in] geometry The arm geometry that will be considered in the
  *   forward kinematic calculations.
  * \param[in] joint_trajectory The joint space trajectory for which the tool
  *   space trajectory will be calculated.
  * \param[out] tool_trajectory The tool space trajectory that results from
  *   the forward kinematic calculations.
  * \return The number of converted trajectory points.
  */
ssize_t era_kinematics_forward_trajectory(
  era_geometry_p geometry,
  era_joint_trajectory_p joint_trajectory,
  era_tool_trajectory_p tool_trajectory);

/** \brief Calculate inverse kinematics for a single configuration
  * \param[in] geometry The arm geometry that will be considered in the
  *   inverse kinematic calculations.
  * \param[in] tool_config The tool space configuration for which the joint
  *   space configuration will be calculated.
  * \param[out] joint_config The joint space configuration that results from
  *   the inverse kinematic calculations.
  */
void era_kinematics_inverse_config(
  era_geometry_p geometry,
  era_tool_config_p tool_config,
  era_joint_config_p joint_config);

/** \brief Calculate inverse kinematics for a trajectory
  * \note The trajectories should be equal in the number of points.
  * \param[in] geometry The arm geometry that will be considered in the
  *   inverse kinematic calculations.
  * \param[in] tool_trajectory The tool space trajectory for which the joint
  *   space trajectory will be calculated.
  * \param[out] joint_trajectory The joint space trajectory that results from
  *   the inverse kinematic calculations.
  * \return The number of converted trajectory points.
  */
ssize_t era_kinematics_inverse_trajectory(
  era_geometry_p geometry,
  era_tool_trajectory_p tool_trajectory,
  era_joint_trajectory_p joint_trajectory);

/** \brief Calculate and set the tool's yaw angle
  * \param[in,out] tool_config The tool space configuration for which the
  *   yaw angle will be calculated and set.
  */
void era_kinematics_set_config_yaw(
  era_tool_config_p tool_config);

#endif
