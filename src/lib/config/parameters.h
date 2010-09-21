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

#ifndef ERA_PARAMETERS_H
#define ERA_PARAMETERS_H

/** \brief ERA configuration parameters
  * Configuration parameters for the BlueBotics ERA-5/1.
  */

/** \brief Predefined ERA parameter constants
  */
#define ERA_PARAMETER_ARM_SECURITY_FUNC             "security-func"
#define ERA_PARAMETER_ARM_ESTOP_CHANNEL             "estop-channel"
#define ERA_PARAMETER_ARM_SWITCH_CHANNEL            "switch-channel"
#define ERA_PARAMETER_ARM_UPPER_LENGTH              "upper-length"
#define ERA_PARAMETER_ARM_LOWER_LENGTH              "lower-length"
#define ERA_PARAMETER_ARM_TOOL_LENGTH               "tool-length"

#define ERA_PARAMETER_JOINT_MIN_POSITION            "min-pos"
#define ERA_PARAMETER_JOINT_MAX_POSITION            "max-pos"
#define ERA_PARAMETER_JOINT_POSITION_MARGIN         "pos-margin"
#define ERA_PARAMETER_JOINT_MAX_VELOCITY            "max-vel"
#define ERA_PARAMETER_JOINT_MIN_ACCELERATION        "min-accel"
#define ERA_PARAMETER_JOINT_MAX_ACCELERATION        "max-accel"

#endif
