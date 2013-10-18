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

#ifndef ERA_CONTROL_SENSORS_H
#define ERA_CONTROL_SENSORS_H

/** \brief ERA sensor data acquisition
  * The sensor data acquisition thread for the BlueBotics ERA-5/1.
  */

#include <tulibs/thread.h>

#include "base/arm.h"

/** \name Error Codes
  * \brief Predefined sensor error codes
  */
//@{
#define ERA_CONTROL_SENSORS_ERROR_NONE               0
#define ERA_CONTROL_SENSORS_ERROR_START              1
//@}

/** \brief Predefined sensor error descriptions
  */
extern const char* era_control_sensors_errors[];

/** \brief Callback handler definition for sensor reading updates
  */
typedef void (*era_control_sensors_handler_p)(
  era_joint_state_p joint_state,
  era_velocity_state_p vel_state,
  double frequency);

/** \brief Structure defining the sensor thread arguments 
  */
typedef struct era_control_sensors_arg_t {
  era_arm_p arm;                         //!< The arm to acquire readings from.
  thread_mutex_p mutex;                  //!< The access mutex of the arm.

  era_control_sensors_handler_p handler; //!< The reading update handler.
  double timestamp;                      //!< The most recent timestamp in [s].
} era_control_sensors_arg_t, *era_control_sensors_arg_p;

/** \brief Start a sensor data acquisition thread
  * \param[in] thread The sensor data acquisition thread to be started.
  * \param[in] arm The opened arm to acquire the sensor readings from.
  * \param[in] mutex The initialized thread mutex to be locked upon accessing 
  *   the arm.
  * \param[in] handler The callback handler that will receive the sensor
  *   reading updates.
  * \param[in] frequency The cycle frequency of the sensor data acquisition.
  * \return The resulting error code.
  */
int era_control_sensors_start(
  thread_p thread,
  era_arm_p arm,
  thread_mutex_p mutex,
  era_control_sensors_handler_p handler,
  double frequency);

/** \brief Exit a sensor data acquisition thread
  * \param[in] thread The sensor data acquisition thread to be terminated.
  */
void era_control_sensors_exit(
  thread_p thread);

#endif
