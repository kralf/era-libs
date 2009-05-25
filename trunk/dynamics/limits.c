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

#include <base/global.h>

#include "limits.h"

const char* era_dynamics_limits_errors[] = {
  "success",
  "velocity space limits exceeded",
};

void era_dynamics_limits_init(era_dynamics_limits_p limits,
  era_velocity_state_p max_vel, era_acceleration_state_p min_accel, 
  era_acceleration_state_p max_accel) {
  limits->max_vel = *max_vel;

  limits->min_accel = *min_accel;
  limits->max_accel = *max_accel;
}

int era_dynamics_limits_test_velocity_state(era_dynamics_limits_p limits,
  era_velocity_state_p vel_state) {
  int i;
  double* omega = (double*)vel_state;
  double* omega_max = (double*)&limits->max_vel;

  for (i = 0; i < sizeof(era_velocity_state_t)/sizeof(double); ++i) {
    if (fabs(omega[i]) > fabs(omega_max[i]))
      return ERA_DYNAMICS_LIMITS_ERROR_EXCEEDED;
  }

  return ERA_DYNAMICS_LIMITS_ERROR_NONE;
}

int era_dynamics_limits_test_acceleration_state(era_dynamics_limits_p limits,
  era_acceleration_state_p accel_state) {
  int i;
  double* omega_dot = (double*)accel_state;
  double* omega_dot_min = (double*)&limits->min_accel;
  double* omega_dot_max = (double*)&limits->max_accel;

  for (i = 0; i < sizeof(era_acceleration_state_t)/sizeof(double); ++i) {
    if ((omega_dot[i] < omega_dot_min[i]) ||
      (omega_dot[i] > omega_dot_max[i]))
      return ERA_DYNAMICS_LIMITS_ERROR_EXCEEDED;
  }

  return ERA_DYNAMICS_LIMITS_ERROR_NONE;
}

ssize_t era_dynamics_limits_test_velocity_profile(era_dynamics_limits_p limits,
  era_velocity_profile_p profile) {
  profile->num_limit_errors = 0;

  int i;
  for (i = 0; i < profile->num_points; ++i) {
    profile->limit_errors[i] = era_dynamics_limits_test_velocity_state(
      limits, &profile->points[i]);
    profile->num_limit_errors +=
      (profile->limit_errors[i] != ERA_DYNAMICS_LIMITS_ERROR_NONE);
  }

  return profile->num_limit_errors;
}
