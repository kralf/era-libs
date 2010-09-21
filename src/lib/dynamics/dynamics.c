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

#include "base/global.h"

#include "dynamics/dynamics.h"

void era_dynamics_linear_state(era_joint_state_p start_state,
  era_joint_state_p target_state, double time, era_velocity_state_p
  vel_state) {
  int i;
  double* theta_start = (double*)start_state;
  double* theta_target = (double*)target_state;
  double* omega = (double*)vel_state;

  for (i = 0; i < min(sizeof(era_joint_state_t)/sizeof(double),
    sizeof(era_velocity_state_t)/sizeof(double)); i++)
    omega[i] = (theta_target[i]-theta_start[i])/time;
}

ssize_t era_dynamics_linear_profile(era_joint_path_p trajectory,
  era_velocity_profile_p profile) {
  int i;
  ssize_t num_points = min(trajectory->num_points, profile->num_points);

  if (num_points > 0) {
    era_velocity_init_state(&profile->points[0]);
    profile->timestamps[0] = trajectory->timestamps[0];
  }

  for (i = 1; i < num_points; i++) {
    era_dynamics_linear_state(&trajectory->points[i-1],
      &trajectory->points[i], trajectory->timestamps[i]-
      trajectory->timestamps[i-1], &profile->points[i]);
    profile->timestamps[i] = trajectory->timestamps[i];
  }

  return num_points;
}

double era_dynamics_limit_state(era_joint_state_p start_state,
  era_joint_state_p target_state, era_dynamics_limits_p limits, double
  vel_factor, era_velocity_state_p vel_state) {
  int i;
  double dtheta_max = 0.0, domega_max = 0.0;
  double* theta_start = (double*)start_state;
  double* theta_target = (double*)target_state;
  double* omega = (double*)vel_state;
  double* omega_max = (double*)&limits->max_vel;

  for (i = 0; i < sizeof(era_joint_state_t)/sizeof(double); i++)
    dtheta_max = max(dtheta_max, fabs(theta_target[i]-theta_start[i]));

  if (dtheta_max == 0.0) {
    for (i = 0; i < sizeof(era_velocity_state_t)/sizeof(double); i++)
      omega[i] = 0.0;

    return 0.0;
  }

  for (i = 0; i < min(sizeof(era_joint_state_t)/sizeof(double),
    sizeof(era_velocity_state_t)/sizeof(double)); i++) {
    omega[i] = (theta_target[i]-theta_start[i])/dtheta_max;
    domega_max = max(domega_max, fabs(omega[i])*clip(vel_factor, 0.0, 1.0)*
      omega_max[i]);
  }

  for (i = 0; i < sizeof(era_velocity_state_t)/sizeof(double); i++)
    omega[i] *= domega_max;

  return dtheta_max/domega_max;
}

ssize_t era_dynamics_limit_profile(era_joint_path_p trajectory,
  era_dynamics_limits_p limits, double vel_factor, era_velocity_profile_p
  profile) {
  int i;
  ssize_t num_points = min(trajectory->num_points, profile->num_points);

  if (num_points > 0) {
    era_velocity_init_state(&profile->points[0]);
    profile->timestamps[0] = trajectory->timestamps[0];
  }

  for (i = 1; i < num_points; i++) {
    double dt = era_dynamics_limit_state(&trajectory->points[i-1],
      &trajectory->points[i], limits, vel_factor, &profile->points[i]);
    profile->timestamps[i] = profile->timestamps[i-1]+dt;
  }

  return num_points;
}
