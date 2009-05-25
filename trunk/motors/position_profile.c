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

#include "position_profile.h"

const char* era_motors_position_profile_errors[] = {
  "success",
  "error starting position profile control operation",
};

void era_motors_position_profile_init(era_motors_position_profile_p profile,
  era_joint_state_p target_state, era_velocity_state_p vel_state, 
  era_acceleration_state_p accel_state, era_acceleration_state_p decel_state,
  epos_profile_type_t type) {
  int i;
  epos_position_profile_p profile_a = (epos_position_profile_p)profile;
  double* target_state_a = (double*)target_state;
  double* vel_state_a = (double*)vel_state;
  double* accel_state_a = (double*)accel_state;
  double* decel_state_a = (double*)decel_state;

  for (i = 0; i < sizeof(era_motors_position_profile_t)/
    sizeof(epos_position_profile_t); ++i)
    epos_position_profile_init(&profile_a[i], target_state_a[i],
    vel_state_a[i], accel_state_a[i], decel_state_a[i], type);
}

int era_motors_position_profile_start(era_motors_p motors, 
  era_motors_position_profile_p profile) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;
  epos_position_profile_p profile_a = (epos_position_profile_p)profile;

  int result = EPOS_ERROR_NONE;
  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    result |= epos_position_profile_start(&node_a[i], &profile_a[i]);

  if (result) {
    era_motors_position_profile_stop(motors);
    return ERA_MOTORS_POSITION_PROFILE_ERROR_START;
  }
  else
    return ERA_MOTORS_POSITION_PROFILE_ERROR_NONE;
}

int era_motors_position_profile_wait(era_motors_p motors, double timeout) {
  return era_motors_wait_status(motors, EPOS_PROFILE_STATUS_REACHED, timeout);
}

void era_motors_position_profile_stop(era_motors_p motors) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    epos_control_stop(&node_a[i].control);
}
