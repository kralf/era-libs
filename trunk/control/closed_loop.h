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

#include <thread.h>

#include <base/era.h>
#include <base/trajectory.h>

/** \brief Predefined closed-loop control error codes
  */
#define ERA_CONTROL_CLOSED_LOOP_ERROR_NONE               0
#define ERA_CONTROL_CLOSED_LOOP_ERROR_START              1
#define ERA_CONTROL_CLOSED_LOOP_ERROR_LIMITS             2

/** \brief Predefined closed-loop control error descriptions
  */
extern const char* era_control_closed_loop_errors[];

/** \brief Structure defining the closed-loop controller's thread arguments 
  */
typedef struct era_control_closed_loop_arg_t {
  era_arm_p arm;                    //!< The arm to be controlled.
  thread_mutex_p mutex;             //!< The access mutex of the arm.

  era_trajectory_p trajectory;      //!< The trajectory to be executed.

  int seg_index;                    //!< The recent trajectory segment index.
  double start_time;                //!< The controller's start time.
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
