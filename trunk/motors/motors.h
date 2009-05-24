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

#ifndef ERA_MOTORS_H
#define ERA_MOTORS_H

/** \brief ERA motor nodes implementation
  * Motor nodes implementation for the BlueBotics ERA-5/1.
  */

#include <epos.h>

#include <config/config.h>

/** \brief Predefined motors constants
  */
#define ERA_MOTORS_WAIT_FOREVER             -1.0

/** \brief Predefined motors error codes
  */
#define ERA_MOTORS_ERROR_NONE               0
#define ERA_MOTORS_ERROR_OPEN               1
#define ERA_MOTORS_ERROR_CLOSE              2
#define ERA_MOTORS_ERROR_MOVE               3
#define ERA_MOTORS_ERROR_WAIT_TIMEOUT       4

struct era_security_t;

/** \brief Structure defining the arm's motors
  */
typedef struct era_motors_t {
  epos_node_t shoulder_yaw;                 //!< The shoulder yaw motor node.
  epos_node_t shoulder_roll;                //!< The shoulder roll motor node.
  epos_node_t shoulder_pitch;               //!< The shoulder pitch motor node.

  epos_node_t elbow_pitch;                  //!< The elbow pitch motor node.

  epos_node_t tool_roll;                    //!< The tool roll motor node.
  epos_node_t tool_opening;                 //!< The tool opening motor node.
} era_motors_t, *era_motors_p;

/** \brief Predefined motor error descriptions
  */
extern const char* era_motors_errors[];

/** \brief Initialize the arm's motors
  * \param[in] motors The motors to be initialized.
  * \param[in] can_dev The common CAN communication device of the motors. If
  *   null, a device will be created from default parameters.
  * \param[in] config The motor configuration parameters.
  */
void era_motors_init(
  era_motors_p motors,
  can_device_p can_dev,
  era_config_joint_p config);

/** \brief Destroy the arm's motors
  * \param[in] motors The motors to be destroyed.
  */
void era_motors_destroy(
  era_motors_p motors);

/** \brief Open the motors
  * \param[in] motors The initialized motors to be opened.
  * \return The resulting error code.
  */
int era_motors_open(
  era_motors_p motors);

/** \brief Close the motors
  * \param[in] motors The opened motors to be closed.
  * \return The resulting error code.
  */
int era_motors_close(
  era_motors_p motors);

/** \brief Wait for a motor status
  * \param[in] motors The motors to wait for.
  * \param[in] status The status word mask to be used.
  * \param[in] timeout The timeout of the wait operation in [s].
  *   A negative value will be interpreted as an eternal wait.
  * \return The resulting error code.
  */
int era_motors_wait_status(
  era_motors_p motors,
  short status,
  double timeout);

#endif
