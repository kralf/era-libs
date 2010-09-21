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

#ifndef ERA_ARM_H
#define ERA_ARM_H

/** \brief ERA arm device
  * Arm device representation for the BlueBotics ERA-5/1.
  */

#include "motors/motors.h"
#include "security/security.h"
#include "base/geometry.h"
#include "kinematics/limits.h"
#include "dynamics/limits.h"

/** \brief Structure defining the BlueBotics ERA-5/1
  */
typedef struct era_arm_t {
  era_motors_t motors;                 //!< The motors of the arm.
  era_security_t security;             //!< The arm's security module.

  era_geometry_t geometry;             //!< The geometry of the arm.
  era_kinematics_limits_t kin_limits;  //!< The arm's kinematic limits.
  era_dynamics_limits_t dyn_limits;    //!< The arm's dynamic limits.

  era_config_t config;                 //!< The arm's configuration parameters.
} era_arm_t, *era_arm_p;

#endif
