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

#ifndef ERA_TRAJECTORY_H
#define ERA_TRAJECTORY_H

/** \brief ERA trajectory implementation
  * Trajectory implementation for the BlueBotics ERA-5/1 robot arm which
  * is based on bicubic splines.
  */

#include <tulibs/spline.h>

#include "base/joint.h"
#include "base/velocity.h"
#include "base/acceleration.h"

/** \brief Predefined trajectory error codes
  */
#define ERA_TRAJECTORY_ERROR_NONE                  0
#define ERA_TRAJECTORY_ERROR_FILE_OPEN             1
#define ERA_TRAJECTORY_ERROR_FILE_FORMAT           2
#define ERA_TRAJECTORY_ERROR_FILE_CREATE           3
#define ERA_TRAJECTORY_ERROR_FILE_WRITE            4
#define ERA_TRAJECTORY_ERROR_UNDEFINED             5

/** \brief Predefined trajectory error descriptions
  */
extern const char* era_trajectory_errors[];

/** \brief Structure defining the arm trajectory
  */
typedef struct era_trajectory_t {
  spline_t shoulder_yaw;    //!< The shoulder's yaw spline.
  spline_t shoulder_roll;   //!< The shoulder's roll spline.
  spline_t shoulder_pitch;  //!< The shoulder's pitch spline.

  spline_t elbow_pitch;     //!< The elbow's pitch spline.

  spline_t tool_roll;       //!< The tool's roll spline.
  spline_t tool_opening;    //!< The tool's opening angle spline.
} era_trajectory_t, *era_trajectory_p;

/** \brief Initialize an empty trajectory
  * \param[in] trajectory The trajectory to be initialized.
  */
void era_trajectory_init(
  era_trajectory_p trajectory);

/** \brief Destroy a trajectory
  * \param[in] trajectory The trajectory to be destroyed.
  */
void era_trajectory_destroy(
  era_trajectory_p trajectory);

/** \brief Read trajectory from file
  * \note A trajectory will be allocated and must be destroyed by the caller.
  * \param[in] filename The name of the file containing the trajectory.
  * \param[out] trajectory The read trajectory.
  * \return The number of trajectory segments read from the file or the 
  *   negative error code.
  */
int era_trajectory_read(
  const char* filename,
  era_trajectory_p trajectory);

/** \brief Write trajectory to file
  * \param[in] filename The name of the file the trajectory will be written to.
  * \param[in] trajectory The trajectory to be written.
  * \return The number of trajectory segments written to the file or the 
  *   negative error code.
  */
int era_trajectory_write(
  const char* filename,
  era_trajectory_p trajectory);

/** \brief Evaluate the trajectory at a given time
  * \param[in] trajectory The trajectory to be evaluated.
  * \param[in] time The time at which to evaluate the trajectory.
  * \param[in] seg_index The index of the trajectory segment to start with 
  *   the linear search.
  * \param[out] joint_state The joint space state of the trajectory at the
  *   given time. Can be null.
  * \param[out] vel_state The optivelocity space state of the trajectory at the
  *   given time. Can be null.
  * \param[out] accel_state The acceleration space state of the trajectory at 
  *   the given time. Can be null.
  * \return The index of the evaluated trajectory segment or the negative 
  *   error code.
  */
int era_trajectory_evaluate(
  era_trajectory_p trajectory,
  double time,
  int seg_index,
  era_joint_state_p joint_state,
  era_velocity_state_p vel_state,
  era_acceleration_state_p accel_state);

#endif
