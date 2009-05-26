/***************************************************************************
 *   Copyright (C) 2004 by Ralf Kaestner                                   *
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

#include <stdio.h>

#include <base/era.h>
#include <base/trajectory.h>

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "usage: %s FILE STEPSIZE\n", argv[0]);
    return -1;
  }
  const char* file = argv[1];
  double step_size = atof(argv[2]);

  era_trajectory_t trajectory;
  era_joint_state_t joint_state;
  era_velocity_state_t vel_state;
  era_acceleration_state_t accel_state;

  int result;
  if ((result = era_trajectory_read(file, &trajectory)) < 0)
    fprintf(stderr, "%s\n", era_trajectory_errors[-result]);
  double t = 0.0;
  int i = 0;
  while ((i = era_trajectory_evaluate(&trajectory, t, i, &joint_state,
    &vel_state, &accel_state)) >= 0) {
    fprintf(stdout, "%lf ", t);
    fprintf(stdout, "%lf %lf %lf %lf %lf %lf ", t,   
      joint_state.shoulder_yaw,
      joint_state.shoulder_roll,
      joint_state.shoulder_pitch,
      joint_state.elbow_pitch,
      joint_state.tool_roll,
      joint_state.tool_opening);
    fprintf(stdout, "%lf %lf %lf %lf %lf %lf ", t,   
      vel_state.shoulder_yaw,
      vel_state.shoulder_roll,
      vel_state.shoulder_pitch,
      vel_state.elbow_pitch,
      vel_state.tool_roll,
      vel_state.tool_opening);
    fprintf(stdout, "%lf %lf %lf %lf %lf %lf\n", t,   
      accel_state.shoulder_yaw,
      accel_state.shoulder_roll,
      accel_state.shoulder_pitch,
      accel_state.elbow_pitch,
      accel_state.tool_roll,
      accel_state.tool_opening);

    t += step_size;
  }

  era_trajectory_destroy(&trajectory);
  return 0;
}
