/*	Trayectory generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#include <stdlib.h>
#include <math.h>

#include "velocity_profile.h"
#include "errors.h"

void era_print_velocity_profile(
  FILE* stream,
  const era_arm_velocity_t* arm_velocities,
  const double* timestamps,
  int num_configurations) {
  int i;

  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s  %10s\n",
    "shoulder_yaw",
    "shoulder_roll",
    "shoulder_pitch",
    "ellbow_pitch",
    "tool_roll",
    "tool_opening",
    "time");

  for (i = 0; i < num_configurations; i++) {
    fprintf(stream,
      "%10.2f °/s  %10.2f °/s  %10.2f °/s  %10.2f °/s  %10.2f °/s  %10.2f °/s  %8.4f s\n",
      arm_velocities[i].shoulder_yaw*180/M_PI,
      arm_velocities[i].shoulder_roll*180/M_PI,
      arm_velocities[i].shoulder_pitch*180/M_PI,
      arm_velocities[i].ellbow_pitch*180/M_PI,
      arm_velocities[i].tool_roll*180/M_PI,
      arm_velocities[i].tool_opening*180/M_PI,
      timestamps[i]);
  }
}

int era_test_velocity_profile_limits(
  const era_arm_velocity_t* arm_velocities,
  int num_velocities) {
  int i, result = ERA_ERROR_NONE;

  for (i = 0; (i < num_velocities) && !result; ++i)
    result = era_test_arm_velocity_limits(&arm_velocities[i]);

  return result;
}

void era_velocity_profile(
  const era_arm_configuration_t* arm_trajectory,
  const double* timestamps,
  int num_configurations,
  era_arm_velocity_t* arm_velocities) {
  int i;

  if (num_configurations > 0) era_init_arm_velocity(&arm_velocities[0]);

  for (i = 1; i < num_configurations; i++)
    era_arm_velocity(&arm_trajectory[i], &arm_trajectory[i-1],
    timestamps[i]-timestamps[i-1], &arm_velocities[i]);
}

void era_velocity_profile_hack(
  const era_arm_configuration_t* arm_trajectory,
  double* timestamps,
  int num_configurations,
  double velocity,
  era_arm_velocity_t* arm_velocities) {
 int i;

  if (num_configurations > 0) era_init_arm_velocity(&arm_velocities[0]);

  for (i = 1; i < num_configurations; i++)
    timestamps[i] = timestamps[i-1]+
    era_sync_arm_velocity(&arm_trajectory[i-1], &arm_trajectory[i],
    velocity, &arm_velocities[i]);
}
