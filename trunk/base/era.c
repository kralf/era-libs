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

#include <kinematics/kinematics.h>
#include <motors/position_profile.h>

#include "era.h"

#include "global.h"

const char* era_errors[] = {
  "success",
  "error opening ERA",
  "error closing ERA",
  "error homing ERA",
  "error moving ERA",
  "ERA limit error",
};

param_t era_default_global_params[] = {
  {ERA_PARAMETER_ARM_SECURITY_FUNC, "0"},
  {ERA_PARAMETER_ARM_ESTOP_CHANNEL, "5"},
  {ERA_PARAMETER_ARM_SWITCH_CHANNEL, "6"},
  {ERA_PARAMETER_ARM_UPPER_LENGTH, "0.2305"},
  {ERA_PARAMETER_ARM_LOWER_LENGTH, "0.224"},
  {ERA_PARAMETER_ARM_TOOL_LENGTH, "0.188"},
};

param_t era_default_shoulder_yaw_joint_params[] = {
  {EPOS_PARAMETER_ID, "1"},
  {EPOS_PARAMETER_SENSOR_TYPE, "1"},
  {EPOS_PARAMETER_SENSOR_PULSES, "500"},
  {EPOS_PARAMETER_MOTOR_CURRENT, "2.5"},
  {EPOS_PARAMETER_GEAR_TRANSMISSION, "-460.0"},

  {EPOS_PARAMETER_HOME_METHOD, "2"},
  {EPOS_PARAMETER_HOME_CURRENT, "1.7"},
  {EPOS_PARAMETER_HOME_VELOCITY, "3.0"},
  {EPOS_PARAMETER_HOME_ACCELERATION, "3.0"},
  {EPOS_PARAMETER_HOME_POSITION, "0.0"},

  {ERA_PARAMETER_JOINT_MIN_POSITION, "-70.5"},
  {ERA_PARAMETER_JOINT_MAX_POSITION, "28.1"},
  {ERA_PARAMETER_JOINT_POSITION_MARGIN, "1.0"},
  {ERA_PARAMETER_JOINT_MAX_VELOCITY, "20.0"},
  {ERA_PARAMETER_JOINT_MIN_ACCELERATION, "-20.0"},
  {ERA_PARAMETER_JOINT_MAX_ACCELERATION, "20.0"},
};

param_t era_default_shoulder_roll_joint_params[] = {
  {EPOS_PARAMETER_ID, "2"},
  {EPOS_PARAMETER_SENSOR_TYPE, "1"},
  {EPOS_PARAMETER_SENSOR_PULSES, "500"},
  {EPOS_PARAMETER_MOTOR_CURRENT, "2.5"},
  {EPOS_PARAMETER_GEAR_TRANSMISSION, "-415.0"},

  {EPOS_PARAMETER_HOME_METHOD, "1"},
  {EPOS_PARAMETER_HOME_CURRENT, "1.2"},
  {EPOS_PARAMETER_HOME_VELOCITY, "2.5"},
  {EPOS_PARAMETER_HOME_ACCELERATION, "2.5"},
  {EPOS_PARAMETER_HOME_POSITION, "10.0"},

  {ERA_PARAMETER_JOINT_MIN_POSITION, "0.0"},
  {ERA_PARAMETER_JOINT_MAX_POSITION, "69.7"},
  {ERA_PARAMETER_JOINT_POSITION_MARGIN, "1.0"},
  {ERA_PARAMETER_JOINT_MAX_VELOCITY, "15.0"},
  {ERA_PARAMETER_JOINT_MIN_ACCELERATION, "-15.0"},
  {ERA_PARAMETER_JOINT_MAX_ACCELERATION, "15.0"},
};

param_t era_default_shoulder_pitch_joint_params[] = {
  {EPOS_PARAMETER_ID, "3"},
  {EPOS_PARAMETER_SENSOR_TYPE, "1"},
  {EPOS_PARAMETER_SENSOR_PULSES, "500"},
  {EPOS_PARAMETER_MOTOR_CURRENT, "2.5"},
  {EPOS_PARAMETER_GEAR_TRANSMISSION, "400.0"},

  {EPOS_PARAMETER_HOME_METHOD, "1"},
  {EPOS_PARAMETER_HOME_CURRENT, "1.4"},
  {EPOS_PARAMETER_HOME_VELOCITY, "2.0"},
  {EPOS_PARAMETER_HOME_ACCELERATION, "2.0"},
  {EPOS_PARAMETER_HOME_POSITION, "0.0"},

  {ERA_PARAMETER_JOINT_MIN_POSITION, "-18.7"},
  {ERA_PARAMETER_JOINT_MAX_POSITION, "65.7"},
  {ERA_PARAMETER_JOINT_POSITION_MARGIN, "1.0"},
  {ERA_PARAMETER_JOINT_MAX_VELOCITY, "15.0"},
  {ERA_PARAMETER_JOINT_MIN_ACCELERATION, "-15.0"},
  {ERA_PARAMETER_JOINT_MAX_ACCELERATION, "15.0"},
};

param_t era_default_elbow_pitch_joint_params[] = {
  {EPOS_PARAMETER_ID, "4"},
  {EPOS_PARAMETER_SENSOR_TYPE, "1"},
  {EPOS_PARAMETER_SENSOR_PULSES, "500"},
  {EPOS_PARAMETER_MOTOR_CURRENT, "2.0"},
  {EPOS_PARAMETER_GEAR_TRANSMISSION, "400.0"},

  {EPOS_PARAMETER_HOME_METHOD, "1"},
  {EPOS_PARAMETER_HOME_CURRENT, "1.4"},
  {EPOS_PARAMETER_HOME_VELOCITY, "2.0"},
  {EPOS_PARAMETER_HOME_ACCELERATION, "2.0"},
  {EPOS_PARAMETER_HOME_POSITION, "25.0"},

  {ERA_PARAMETER_JOINT_MIN_POSITION, "0.9"},
  {ERA_PARAMETER_JOINT_MAX_POSITION, "102.5"},
  {ERA_PARAMETER_JOINT_POSITION_MARGIN, "1.0"},
  {ERA_PARAMETER_JOINT_MAX_VELOCITY, "15.0"},
  {ERA_PARAMETER_JOINT_MIN_ACCELERATION, "-15.0"},
  {ERA_PARAMETER_JOINT_MAX_ACCELERATION, "15.0"},
};

param_t era_default_tool_roll_joint_params[] = {
  {EPOS_PARAMETER_ID, "5"},
  {EPOS_PARAMETER_SENSOR_TYPE, "1"},
  {EPOS_PARAMETER_SENSOR_PULSES, "512"},
  {EPOS_PARAMETER_MOTOR_CURRENT, "0.3"},
  {EPOS_PARAMETER_GEAR_TRANSMISSION, "-193.43"},

  {EPOS_PARAMETER_HOME_METHOD, "1"},
  {EPOS_PARAMETER_HOME_CURRENT, "0.15"},
  {EPOS_PARAMETER_HOME_VELOCITY, "15.0"},
  {EPOS_PARAMETER_HOME_ACCELERATION, "15.0"},
  {EPOS_PARAMETER_HOME_POSITION, "-10.0"},

  {ERA_PARAMETER_JOINT_MIN_POSITION, "-158.0"},
  {ERA_PARAMETER_JOINT_MAX_POSITION, "150.0"},
  {ERA_PARAMETER_JOINT_POSITION_MARGIN, "1.0"},
  {ERA_PARAMETER_JOINT_MAX_VELOCITY, "60.0"},
  {ERA_PARAMETER_JOINT_MIN_ACCELERATION, "-60.0"},
  {ERA_PARAMETER_JOINT_MAX_ACCELERATION, "60.0"},
};

param_t era_default_tool_opening_joint_params[] = {
  {EPOS_PARAMETER_ID, "6"},
  {EPOS_PARAMETER_SENSOR_TYPE, "1"},
  {EPOS_PARAMETER_SENSOR_PULSES, "512"},
  {EPOS_PARAMETER_MOTOR_CURRENT, "0.3"},
  {EPOS_PARAMETER_GEAR_TRANSMISSION, "-137.75"},

  {EPOS_PARAMETER_HOME_METHOD, "-1"},
  {EPOS_PARAMETER_HOME_CURRENT, "0.1"},
  {EPOS_PARAMETER_HOME_VELOCITY, "10.0"},
  {EPOS_PARAMETER_HOME_ACCELERATION, "10.0"},
  {EPOS_PARAMETER_HOME_POSITION, "0.0"},

  {ERA_PARAMETER_JOINT_MIN_POSITION, "-65.3"},
  {ERA_PARAMETER_JOINT_MAX_POSITION, "27.0"},
  {ERA_PARAMETER_JOINT_POSITION_MARGIN, "1.0"},
  {ERA_PARAMETER_JOINT_MAX_VELOCITY, "30.0"},
  {ERA_PARAMETER_JOINT_MIN_ACCELERATION, "-30.0"},
  {ERA_PARAMETER_JOINT_MAX_ACCELERATION, "30.0"},
};

era_config_t era_default_config = {
  { era_default_global_params,
    sizeof(era_default_global_params)/sizeof(param_t) },
  { 
    { era_default_shoulder_yaw_joint_params,
      sizeof(era_default_shoulder_yaw_joint_params)/sizeof(param_t) },
    { era_default_shoulder_roll_joint_params,
      sizeof(era_default_shoulder_roll_joint_params)/sizeof(param_t) },
    { era_default_shoulder_pitch_joint_params,
      sizeof(era_default_shoulder_pitch_joint_params)/sizeof(param_t) },
    { era_default_elbow_pitch_joint_params,
      sizeof(era_default_elbow_pitch_joint_params)/sizeof(param_t) },
    { era_default_tool_roll_joint_params,
      sizeof(era_default_tool_roll_joint_params)/sizeof(param_t) },
    { era_default_tool_opening_joint_params,
      sizeof(era_default_tool_opening_joint_params)/sizeof(param_t) },
  },
};

void era_init(era_arm_p arm, can_device_p can_dev, era_config_p config) {
  era_config_init_default(&arm->config, &era_default_config);
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
    EPOS_CONFIG_ARG_PREFIX);
    
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






// int era_read(
//   era_read_handler_t handler,
//   double frequency) {
//   int result = era_sensors_start(handler, frequency);
// 
//   if (!result) era_thread_wait_exit(&era_sensors_thread);
// 
//   return result;
// }
// 
// int era_set_configuration(
//   const era_arm_configuration_t* configuration,
//   const era_arm_velocity_t* velocity) {
//   int result;
// 
//   result = era_test_arm_configuration_limits(configuration);
//   if (!result) result = era_test_arm_velocity_limits(velocity);
// 
//   if (!result) {
//     era_motor_configuration_t motor_configuration;
//     era_motor_velocity_t motor_velocity;
//     era_arm_to_motor(configuration, velocity, &motor_configuration,
//       &motor_velocity);
// 
//     pthread_mutex_lock(&era_mutex);
// 
//     if (configuration)
//       result = era_motors_set_configuration(&motor_configuration,
//       &motor_velocity);
//     else
//       result = era_motors_set_configuration(0, &motor_velocity);
// 
//     pthread_mutex_unlock(&era_mutex);
//   }
// 
//   return result;
// }
// 
// int era_move(
//   const era_arm_configuration_t* target,
//   double velocity,
//   int wait) {
//   int result;
// 
//   era_arm_configuration_t current;
//   era_arm_velocity_t arm_velocity;
// 
//   era_get_configuration(&current, 0);
//   era_sync_arm_velocity(&current, target, velocity, &arm_velocity);
// 
//   result = era_set_configuration(target, &arm_velocity);
// 
//   if (!result && wait) era_motors_wait(ERA_MOTORS_WAIT_TARGET_REACHED);
// 
//   return result;
// }
// 
// int era_move_home(
//   double velocity,
//   int wait) {
//   return era_move(&era_home, velocity, wait);
// }
// 
// int era_move_tool(
//   const era_tool_configuration_t* target,
//   double velocity,
//   int wait) {
//   era_arm_configuration_t arm_configuration;
// 
//   era_inverse_kinematics(target, &arm_configuration);
// 
//   return era_move(&arm_configuration, velocity, wait);
// }
// 
// int era_move_trajectory(
//   const era_arm_configuration_t* trajectory,
//   const double* timestamps,
//   int num_configurations) {
//   int result;
// 
//   result = era_test_trajectory_limits(trajectory, num_configurations, 0);
// 
//   if (!result) {
//     era_arm_velocity_t velocities[num_configurations];
// 
//     era_velocity_profile(trajectory, timestamps, num_configurations,
//       velocities);
// 
//     result = era_test_velocity_profile_limits(velocities, num_configurations);
// 
//     if (!result) {
//       result = era_move(&trajectory[0], ERA_DEFAULT_VELOCITY, 1);
// 
//       if (!result)
//         result = era_move_velocity_profile(velocities, timestamps,
//         num_configurations);
//     }
//   }
// 
//   return result;
// }
// 
// int era_move_tool_trajectory(
//   const era_tool_configuration_t* trajectory,
//   const double* timestamps,
//   int num_configurations) {
//   era_arm_configuration_t arm_trajectory[num_configurations];
// 
//   era_trajectory_inverse_kinematics(trajectory, num_configurations,
//     arm_trajectory);
// 
//   return era_move_trajectory(arm_trajectory, timestamps, num_configurations);
// }
// 
// int era_move_velocity_profile(
//   const era_arm_velocity_t* arm_velocities,
//   const double* timestamps,
//   int num_velocities) {
//   int result =
//     era_controller_start(arm_velocities, timestamps, num_velocities);
// 
//   if (!result) era_thread_wait_exit(&era_controller_thread);
// 
//   return result;
// }
// 
// double era_get_configuration_error(
//   const era_arm_configuration_t* target) {
//   int i;
//   double squared_error = 0.0;
// 
//   era_arm_configuration_t current;
//   double* current_conf = (double*)&current;
//   double* target_conf = (double*)target;
// 
//   era_get_configuration(&current, 0);
// 
//   for (i = 0; i < sizeof(era_arm_configuration_t)/sizeof(double); i++)
//     squared_error += sqr(target_conf[i]-current_conf[i]);
// 
//   return sqrt(squared_error);
// }
