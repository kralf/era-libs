/*	Trayectory generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#include <stdlib.h>
#include <math.h>

#include "trajectory.h"
#include "errors.h"

void era_print_tool_trajectory(
  FILE* stream,
  const era_tool_configuration_t* tool_trajectory,
  int num_configurations) {
  int i;

  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s\n",
    "x",
    "y",
    "z",
    "yaw",
    "roll",
    "opening");

  for (i = 0; i < num_configurations; i++) {
    fprintf(stream,
      "%12.4f m  %12.4f m  %12.4f m  %12.2f °  %12.2f °  %12.2f °\n",
      tool_trajectory[i].x,
      tool_trajectory[i].y,
      tool_trajectory[i].z,
      tool_trajectory[i].yaw*180/M_PI,
      tool_trajectory[i].roll*180/M_PI,
      tool_trajectory[i].opening*180/M_PI);
  }
}

void era_print_arm_trajectory(
  FILE* stream,
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations) {
  int i;

  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s\n",
    "shoulder_yaw",
    "shoulder_roll",
    "shoulder_pitch",
    "ellbow_pitch",
    "tool_roll",
    "tool_opening");

  for (i = 0; i < num_configurations; i++) {
    fprintf(stream,
      "%12.2f °  %12.2f °  %12.2f °  %12.2f °  %12.2f °  %12.2f °\n",
      arm_trajectory[i].shoulder_yaw*180/M_PI,
      arm_trajectory[i].shoulder_roll*180/M_PI,
      arm_trajectory[i].shoulder_pitch*180/M_PI,
      arm_trajectory[i].ellbow_pitch*180/M_PI,
      arm_trajectory[i].tool_roll*180/M_PI,
      arm_trajectory[i].tool_opening*180/M_PI);
  }
}

int era_test_trajectory_limits(
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations) {
  int i, result = ERA_ERROR_NONE;

  for (i = 0; (i < num_configurations) && !result; ++i)
    result = era_test_arm_configuration_limits(&arm_trajectory[i]);

  return result;
}

void era_trajectory_forward_kinematics(
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations,
  era_tool_configuration_t* tool_trajectory) {
  int i;

  for (i = 0; i < num_configurations; i++)
    era_forward_kinematics(&arm_trajectory[i], &tool_trajectory[i]);
}

void era_trajectory_inverse_kinematics(
  const era_tool_configuration_t* tool_trajectory,
  int num_configurations,
  era_arm_configuration_t* arm_trajectory) {
  int i;

  for (i = 0; i < num_configurations; i++)
    era_inverse_kinematics(&tool_trajectory[i], &arm_trajectory[i]);
}
