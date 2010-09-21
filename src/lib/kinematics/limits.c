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

#include "kinematics/limits.h"

const char* era_kinematics_limits_errors[] = {
  "success",
  "joint space limits exceeded",
  "singular state",
};

void era_kinematics_limits_init(era_kinematics_limits_p limits,
  era_joint_state_p min, era_joint_state_p max, era_joint_state_p margin) {
  limits->min = *min;
  limits->max = *max;

  limits->margin = *margin;
}

int era_kinematics_limits_test_state(era_kinematics_limits_p limits,
  era_joint_state_p state) {
  int i;
  double* theta = (double*)state;
  double* theta_min = (double*)&limits->min;
  double* theta_max = (double*)&limits->max;
  double* theta_margin = (double*)&limits->margin;

  for (i = 0; i < sizeof(era_joint_state_t)/sizeof(double); ++i) {
    if (theta[i] != theta[i])
      return ERA_KINEMATICS_LIMITS_ERROR_SINGULARITY;

    if ((theta[i] < theta_min[i]+theta_margin[i]) ||
      (theta[i] > theta_max[i]-theta_margin[i]))
      return ERA_KINEMATICS_LIMITS_ERROR_EXCEEDED;
  }

  return ERA_KINEMATICS_LIMITS_ERROR_NONE;
}

ssize_t era_kinematics_limits_test_path(era_kinematics_limits_p limits,
  era_joint_path_p path) {
  path->num_limit_errors = 0;

  int i;
  for (i = 0; i < path->num_points; ++i) {
    path->limit_errors[i] = era_kinematics_limits_test_state(limits,
      &path->points[i]);
    path->num_limit_errors +=
      (path->limit_errors[i] != ERA_KINEMATICS_LIMITS_ERROR_NONE);
  }

  return path->num_limit_errors;
}
