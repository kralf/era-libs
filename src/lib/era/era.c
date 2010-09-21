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

#include <stdlib.h>
#include <string.h>

#include "base/global.h"
#include "kinematics/kinematics.h"
#include "motors/position_profile.h"

#include "era.h"

const char* era_errors[] = {
  "success",
  "error opening ERA",
  "error closing ERA",
  "error homing ERA",
  "error moving ERA",
  "ERA limit error",
};

void era_init(era_arm_p arm, can_device_p can_dev, era_config_p config) {
  era_config_init_default(&arm->config, &era_config_default);
  if (config)
    era_config_set(&arm->config, config);

  era_motors_init(&arm->motors, can_dev, &arm->config.joints);
  era_security_init(&arm->security,
    config_get_int(&arm->config.arm, ERA_PARAMETER_ARM_SECURITY_FUNC),
    config_get_int(&arm->config.arm, ERA_PARAMETER_ARM_ESTOP_CHANNEL),
    config_get_int(&arm->config.arm, ERA_PARAMETER_ARM_SWITCH_CHANNEL));

  era_joint_state_t min_state, max_state, state_margin;
  era_velocity_state_t max_vel;
  era_acceleration_state_t min_accel, max_accel;
  era_geometry_init(&arm->geometry,
    config_get_float(&arm->config.arm, ERA_PARAMETER_ARM_UPPER_LENGTH),
    config_get_float(&arm->config.arm, ERA_PARAMETER_ARM_LOWER_LENGTH),
    config_get_float(&arm->config.arm, ERA_PARAMETER_ARM_TOOL_LENGTH));
  era_kinematics_limits_init(&arm->kin_limits,
    (era_joint_state_p)era_config_joint_get_rad(&arm->config, 
      ERA_PARAMETER_JOINT_MIN_POSITION, (double*)&min_state),
    (era_joint_state_p)era_config_joint_get_rad(&arm->config, 
      ERA_PARAMETER_JOINT_MAX_POSITION, (double*)&max_state),
    (era_joint_state_p)era_config_joint_get_rad(&arm->config, 
      ERA_PARAMETER_JOINT_POSITION_MARGIN, (double*)&state_margin));
  era_dynamics_limits_init(&arm->dyn_limits,
    (era_velocity_state_p)era_config_joint_get_rad(&arm->config, 
      ERA_PARAMETER_JOINT_MAX_VELOCITY, (double*)&max_vel),
    (era_acceleration_state_p)era_config_joint_get_rad(&arm->config, 
      ERA_PARAMETER_JOINT_MIN_ACCELERATION, (double*)&min_accel),
    (era_acceleration_state_p)era_config_joint_get_rad(&arm->config, 
      ERA_PARAMETER_JOINT_MAX_ACCELERATION, (double*)&max_accel));
}

void era_init_arg(era_arm_p arm, int argc, char **argv, const char* prefix) {
  can_device_p can_dev = malloc(sizeof(can_device_t));
  can_init_arg(can_dev, argc, argv, 0);

  era_config_t config;
  era_config_init_arg(&config, argc, argv, (prefix) ? prefix : 
    ERA_CONFIG_ARG_PREFIX);
    
  era_init(arm, can_dev, &config);

  era_config_destroy(&config);
}

void era_destroy(era_arm_p arm) {
  era_motors_destroy(&arm->motors);

  era_config_destroy(&arm->config);
}

int era_open(era_arm_p arm) {
  if (!era_motors_open(&arm->motors) &&
    !era_security_enable(&arm->security, &arm->motors))
    return ERA_ERROR_NONE;
  else
    return ERA_ERROR_OPEN;
}

int era_close(era_arm_p arm) {
  if (!era_security_disable(&arm->security, &arm->motors) &&
    !era_motors_close(&arm->motors))
    return ERA_ERROR_NONE;
  else
    return ERA_ERROR_CLOSE;
}

void era_print_state(FILE* stream, era_arm_p arm) {
  era_tool_state_t tool_state;
  era_joint_state_t joint_state;
  era_velocity_state_t vel_state;

  era_get_joint_state(arm, &joint_state);
  era_kinematics_forward_state(&arm->geometry, &joint_state, &tool_state);
  era_get_velocity_state(arm, &vel_state);

  fprintf(stream, "%14s: % 8.2f deg  % 8.2f deg/s  %7s: % 8.4f m\n",
    "shoulder_yaw", rad_to_deg(joint_state.shoulder_yaw),
    rad_to_deg(vel_state.shoulder_yaw), "x", tool_state.x);
  fprintf(stream, "%14s: % 8.2f deg  % 8.2f deg/s  %7s: % 8.4f m\n",
    "shoulder_roll", rad_to_deg(joint_state.shoulder_roll),
    rad_to_deg(vel_state.shoulder_roll), "y", tool_state.y);
  fprintf(stream, "%14s: % 8.2f deg  % 8.2f deg/s  %7s: % 8.4f m\n",
    "shoulder_pitch", rad_to_deg(joint_state.shoulder_pitch),
    rad_to_deg(vel_state.shoulder_pitch), "z", tool_state.z);
  fprintf(stream, "%14s: % 8.2f deg  % 8.2f deg/s  %7s: % 8.2f deg\n",
    "elbow_pitch", rad_to_deg(joint_state.elbow_pitch),
    rad_to_deg(vel_state.elbow_pitch), "yaw", tool_state.yaw);
  fprintf(stream, "%14s: % 8.2f deg  % 8.2f deg/s  %7s: % 8.2f deg\n",
    "tool_roll", rad_to_deg(joint_state.tool_roll),
    rad_to_deg(vel_state.tool_roll), "roll", rad_to_deg(tool_state.roll));
  fprintf(stream, "%14s: % 8.2f deg  % 8.2f deg/s  %7s: % 8.2f deg\n",
    "tool_opening", rad_to_deg(joint_state.tool_opening),
    rad_to_deg(vel_state.tool_opening), "opening", 
    rad_to_deg(tool_state.opening));
}

void era_get_tool_state(era_arm_p arm, era_tool_state_p tool_state) {
  era_joint_state_t joint_state;
  era_get_joint_state(arm, &joint_state);
  
  era_kinematics_forward_state(&arm->geometry, &joint_state, tool_state);
}

void era_get_joint_state(era_arm_p arm, era_joint_state_p joint_state) {
  int i;
  epos_node_p node_a = (epos_node_p)&arm->motors;
  double* joint_state_a = (double*)joint_state;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    joint_state_a[i] = epos_get_position(&node_a[i]);
}

void era_get_velocity_state(era_arm_p arm, era_velocity_state_p vel_state) {
  int i;
  epos_node_p node_a = (epos_node_p)&arm->motors;
  double* vel_state_a = (double*)vel_state;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    vel_state_a[i] = epos_get_velocity(&node_a[i]);
}

int era_home(era_arm_p arm) {
  if (!era_motors_home_start(&arm->motors, &arm->security))
    return era_motors_home_wait(&arm->motors, ERA_MOTORS_WAIT_FOREVER);
  else
    return ERA_ERROR_HOME;
}

int era_move_joints(era_arm_p arm, era_joint_state_p target_state, double 
  vel_factor) {
  era_joint_state_t start_state;
  era_velocity_state_t vel_state;

  if (era_kinematics_limits_test_state(&arm->kin_limits, target_state))
    return ERA_ERROR_LIMITS;

  era_get_joint_state(arm, &start_state);
  era_dynamics_limit_state(&start_state, target_state, &arm->dyn_limits, 
    vel_factor, &vel_state);

  era_motors_position_profile_t profile;
  era_motors_position_profile_init(&profile, target_state, &vel_state,
    &arm->dyn_limits.max_accel, &arm->dyn_limits.min_accel, 
    epos_profile_sinusoidal);

  if (!era_motors_position_profile_start(&arm->motors, &profile))
    return era_motors_position_profile_wait(&arm->motors, 
      ERA_MOTORS_WAIT_FOREVER);
  else
    return ERA_ERROR_MOVE;
}

int era_move_tool(era_arm_p arm, era_tool_state_p tool_state, double 
  vel_factor) {
  era_joint_state_t joint_state;
  era_kinematics_inverse_state(&arm->geometry, tool_state, &joint_state);

  return era_move_joints(arm, &joint_state, vel_factor);
}

int era_move_home(era_arm_p arm, double vel_factor) {
  int i;
  era_joint_state_t home_state;
  double* home_state_a = (double*)&home_state;
  config_p config_a = (config_p)&arm->config.joints;

  for (i = 0; i < sizeof(era_joint_state_t)/sizeof(double); ++i)
    home_state_a[i] = deg_to_rad(config_get_float(&config_a[i], 
    EPOS_PARAMETER_HOME_POSITION));

  return era_move_joints(arm, &home_state, vel_factor);
}
