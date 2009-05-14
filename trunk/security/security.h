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

#ifndef ERA_SECURITY_H
#define ERA_SECURITY_H

/** \brief ERA security functions
  * Security functions for the BlueBotics ERA-5/1.
  */

/** \brief Predefined ERA security codes
  */
#define ERA_SECURITY_ERROR_NONE               0
#define ERA_SECURITY_ERROR_ENABLE             1
#define ERA_SECURITY_ERROR_DISABLE            2

struct era_motors_t;

/** \brief Security functionalities
  */
typedef enum {
  era_security_neg_switch_pos_estop = 0,
  era_security_pos_switch_neg_estop = 1,
} era_security_func_t;

/** \brief Structure defining the arm's security module
  */
typedef struct era_security_t {
  era_security_func_t func;     //!< The security functionality.

  int estop_channel;            //!< The emergency stop channel.
  int switch_channel;           //!< The limit switch channel.
} era_security_t, *era_security_p;

/** \brief Predefined security error descriptions
  */
extern const char* era_security_errors[];

/** \brief Initialize the arm's security module
  * \param[in] security The security module to be initialized.
  * \param[in] func The security functionality.
  * \param[in] estop_channel The emergency stop channel.
  * \param[in] switch_channel The limit switch channel.
  */
void era_security_init(
  era_security_p security,
  era_security_func_t func,
  int estop_channel,
  int switch_channel);

/** \brief Fully enable security module
  * \param[in] security The initialized security module to be enabled.
  * \param[in] motors The motors to enable security for.
  * \return The resulting error code.
  */
int era_security_enable(
  era_security_p security,
  struct era_motors_t* motors);

/** \brief Fully disable security module
  * \param[in] security The initialized security module to be disabled.
  * \param[in] motors The motors to disable security for.
  * \return The resulting error code.
  */
int era_security_disable(
  era_security_p security,
  struct era_motors_t* motors);

/** \brief Enable security module for homing
  * \param[in] security The initialized security module to be enabled.
  * \param[in] motors The motors to enable security for.
  * \return The resulting error code.
  */
int era_security_enable_home(
  era_security_p security,
  struct era_motors_t* motors);

#endif
