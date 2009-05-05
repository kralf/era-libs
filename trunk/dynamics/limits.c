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
  era_velocity_config_p max_vel) {
  limits->max_vel = *max_vel;
}

int era_dynamics_limits_test_config(era_dynamics_limits_p limits,
  era_velocity_config_p config) {
  int i;
  double* omega = (double*)config;
  double* omega_max = (double*)&limits->max_vel;

  for (i = 0; i < sizeof(era_velocity_config_t)/sizeof(double); ++i) {
    if (abs(omega[i]) > abs(omega_max[i]))
      return ERA_DYNAMICS_LIMITS_ERROR_EXCEEDED;
  }

  return ERA_DYNAMICS_LIMITS_ERROR_NONE;
}

ssize_t era_dynamics_limits_test_profile(era_dynamics_limits_p limits,
  era_velocity_profile_p profile) {
  profile->num_limit_errors = 0;

  int i;
  for (i = 0; i < profile->num_points; ++i) {
    profile->limit_errors[i] = era_dynamics_limits_test_config(limits,
      &profile->points[i]);
    profile->num_limit_errors +=
      (profile->limit_errors[i] != ERA_DYNAMICS_LIMITS_ERROR_NONE);
  }

  return profile->num_limit_errors;
}
