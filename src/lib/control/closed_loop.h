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

#ifndef ERA_CONTROL_CLOSED_LOOP_H
#define ERA_CONTROL_CLOSED_LOOP_H

/** \brief ERA closed-loop trajectory control
  * A closed-loop trajectory controller for the BlueBotics ERA-5/1.
  */

#include <tulibs/thread.h>

#include "base/arm.h"
#include "base/trajectory.h"

/** \name Constants
  * \brief Predefined closed-loop control constants
  */
//@{
#define ERA_CONTROL_CLOSED_LOOP_PARAMETER_P         "p-gain"
#define ERA_CONTROL_CLOSED_LOOP_PARAMETER_I         "i-gain"
//@}

/** \name Error Codes
  * \brief Predefined closed-loop control error codes
  */
//@{
#define ERA_CONTROL_CLOSED_LOOP_ERROR_NONE          0
#define ERA_CONTROL_CLOSED_LOOP_ERROR_START         1
#define ERA_CONTROL_CLOSED_LOOP_ERROR_LIMITS        2
//@}

/** \brief Predefined closed-loop control error descriptions
  */
extern const char* era_control_closed_loop_errors[];

/** \brief Structure defining a closed-loop controller's P-gain
  */
typedef struct era_control_closed_loop_p_t {
  double shoulder_yaw;              //!< The shoulder yaw P-gain.
  double shoulder_roll;             //!< The shoulder roll P-gain.
  double shoulder_pitch;            //!< The shoulder pitch P-gain.

  double elbow_pitch;               //!< The elbow pitch P-gain.

  double tool_roll;                 //!< The tool roll P-gain.
  double tool_opening;              //!< The tool opening P-gain.
} era_control_closed_loop_p_t, *era_control_closed_loop_p_p;

/** \brief Structure defining a closed-loop controller's P-gain
  */
typedef struct era_control_closed_loop_i_t {
  double shoulder_yaw;              //!< The shoulder yaw I-gain.
  double shoulder_roll;             //!< The shoulder roll I-gain.
  double shoulder_pitch;            //!< The shoulder pitch I-gain.

  double elbow_pitch;               //!< The elbow pitch I-gain.

  double tool_roll;                 //!< The tool roll I-gain.
  double tool_opening;              //!< The tool opening I-gain.
} era_control_closed_loop_i_t, *era_control_closed_loop_i_p;

/** \brief Structure defining the closed-loop controller's thread arguments 
  */
typedef struct era_control_closed_loop_arg_t {
  era_arm_p arm;                    //!< The arm to be controlled.
  thread_mutex_p mutex;             //!< The access mutex of the arm.

  era_trajectory_p trajectory;      //!< The trajectory to be executed.

  era_control_closed_loop_p_t p;    //!< The controller's P-gain.
  era_control_closed_loop_i_t i;    //!< The controller's I-gain.

  int seg_index;                    //!< The recent trajectory segment index.
  double start_time;                //!< The controller's start time in [s].

  era_joint_state_t error;          //!< The recent control error.
  era_velocity_state_t output;      //!< The recent control output.
  double timestamp;                 //!< The most recent timestamp in [s].
} era_control_closed_loop_arg_t, *era_control_closed_loop_arg_p;

/** \brief Start a closed-loop control thread
  * \param[in] thread The closed-loop control thread to be started.
  * \param[in] arm The opened arm to be controlled.
  * \param[in] mutex The initialized thread mutex to be locked upon accessing 
  *   the arm.
  * \param[in] trajectory The trajectory to be executed by the closed-loop 
  *   controller.
  * \param[in] frequency The cycle frequency of the closed-loop control thread.
  * \return The resulting error code.
  */
int era_control_closed_loop_start(
  thread_p thread,
  era_arm_p arm,
  thread_mutex_p mutex,
  era_trajectory_p trajectory,
  double frequency);

/** \brief Exit an closed-loop control thread
  * \param[in] thread The closed-loop control thread to be terminated.
  */
void era_control_closed_loop_exit(
  thread_p thread);

#endif
