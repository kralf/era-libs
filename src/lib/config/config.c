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

#include <string.h>

#include <libepos/epos.h>

#include "base/global.h"

#include "config/config.h"

const char* era_config_joint_prefixes[] = {
  ERA_CONFIG_ARG_PREFIX_SHOULDER_YAW,
  ERA_CONFIG_ARG_PREFIX_SHOULDER_ROLL,
  ERA_CONFIG_ARG_PREFIX_SHOULDER_PITCH,
  ERA_CONFIG_ARG_PREFIX_ELBOW_PITCH,
  ERA_CONFIG_ARG_PREFIX_TOOL_ROLL,
  ERA_CONFIG_ARG_PREFIX_TOOL_OPENING,
};

param_t era_config_default_global_params[] = {
  {ERA_PARAMETER_ARM_SECURITY_FUNC, "0"},
  {ERA_PARAMETER_ARM_ESTOP_CHANNEL, "5"},
  {ERA_PARAMETER_ARM_SWITCH_CHANNEL, "6"},
  {ERA_PARAMETER_ARM_UPPER_LENGTH, "0.2305"},
  {ERA_PARAMETER_ARM_LOWER_LENGTH, "0.224"},
  {ERA_PARAMETER_ARM_TOOL_LENGTH, "0.188"},
};

param_t era_config_default_shoulder_yaw_joint_params[] = {
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

param_t era_config_default_shoulder_roll_joint_params[] = {
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

param_t era_config_default_shoulder_pitch_joint_params[] = {
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

param_t era_config_default_elbow_pitch_joint_params[] = {
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

param_t era_config_default_tool_roll_joint_params[] = {
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

param_t era_config_default_tool_opening_joint_params[] = {
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

era_config_t era_config_default = {
  { era_config_default_global_params,
    sizeof(era_config_default_global_params)/sizeof(param_t) },
  {
    { era_config_default_shoulder_yaw_joint_params,
      sizeof(era_config_default_shoulder_yaw_joint_params)/sizeof(param_t) },
    { era_config_default_shoulder_roll_joint_params,
      sizeof(era_config_default_shoulder_roll_joint_params)/sizeof(param_t) },
    { era_config_default_shoulder_pitch_joint_params,
      sizeof(era_config_default_shoulder_pitch_joint_params)/sizeof(param_t) },
    { era_config_default_elbow_pitch_joint_params,
      sizeof(era_config_default_elbow_pitch_joint_params)/sizeof(param_t) },
    { era_config_default_tool_roll_joint_params,
      sizeof(era_config_default_tool_roll_joint_params)/sizeof(param_t) },
    { era_config_default_tool_opening_joint_params,
      sizeof(era_config_default_tool_opening_joint_params)/sizeof(param_t) },
  },
};

void era_config_init(era_config_p config) {
  config_init(&config->arm);

  config_p joint_config_a = (config_p)&config->joints;
  int i;
  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    config_init(&joint_config_a[i]);
}

void era_config_init_default(era_config_p config, era_config_p 
  default_config) {
  config_init_default(&config->arm, &default_config->arm);

  config_p joint_config_a = (config_p)&config->joints;
  config_p default_joint_config_a = (config_p)&default_config->joints;
  int i;
  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    config_init_default(&joint_config_a[i], &default_joint_config_a[i]);
}

void era_config_init_arg(era_config_p config, int argc, char **argv, 
  const char* prefix) {
  char arm_prefix[256];
  sprintf(arm_prefix, "%s-%s", (prefix) ? prefix : ERA_CONFIG_ARG_PREFIX, 
    ERA_CONFIG_ARG_PREFIX_ARM);
  char joint_prefixes[sizeof(era_config_joint_t)/sizeof(config_t)][256];
  int i;
  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    sprintf(joint_prefixes[i], "%s-%s", (prefix) ? prefix :
      ERA_CONFIG_ARG_PREFIX, era_config_joint_prefixes[i]);

  if (config_init_arg(&config->arm, argc, argv, arm_prefix)) {
    config_print_help(stdout, &can_default_config, CAN_CONFIG_ARG_PREFIX);
    config_print_help(stdout, &era_config_default.arm, arm_prefix);
    
    config_p default_joint_config_a = (config_p)&era_config_default.joints;
    for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
      config_print_help(stdout, &default_joint_config_a[i], joint_prefixes[i]);
    exit(0);
  }
    
  config_init_arg(&config->arm, argc, argv, arm_prefix);

  config_p joint_config_a = (config_p)&config->joints;
  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i) {
    char joint_prefix[256];
    sprintf(joint_prefix, "%s-%s", (prefix) ? prefix : ERA_CONFIG_ARG_PREFIX, 
      era_config_joint_prefixes[i]);

    config_init_arg(&joint_config_a[i], argc, argv, joint_prefix);
  }
}

void era_config_destroy(era_config_p config) {
  config_destroy(&config->arm);

  config_p joint_config_a = (config_p)&config->joints;
  int i;
  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    config_destroy(&joint_config_a[i]);
}

void era_config_print(FILE* stream, era_config_p config) {
  config_print(stream, &config->arm);

  config_p joint_config_a = (config_p)&config->joints;
  int i;
  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    config_print(stream, &joint_config_a[i]);
}

void era_config_set(era_config_p dst_config, era_config_p src_config) {
  config_set(&dst_config->arm, &src_config->arm);

  config_p src_joint_config_a = (config_p)&src_config->joints;
  config_p dst_joint_config_a = (config_p)&dst_config->joints;
  int i;
  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    config_set(&dst_joint_config_a[i], &src_joint_config_a[i]);
}

const char** era_config_joint_get_string(era_config_p config, const char* key, 
  const char** values) {
  config_p joint_config_a = (config_p)&config->joints;
  int i;

  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    values[i] = config_get_string(&joint_config_a[i], key);

  return values;
}

int* era_config_joint_get_int(era_config_p config, const char* key, 
  int* values) {
  config_p joint_config_a = (config_p)&config->joints;
  int i;

  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    values[i] = config_get_int(&joint_config_a[i], key);

  return values;
}

double* era_config_joint_get_float(era_config_p config, const char* key, 
  double* values) {
  config_p joint_config_a = (config_p)&config->joints;
  int i;

  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    values[i] = config_get_float(&joint_config_a[i], key);

  return values;
}

double* era_config_joint_get_rad(era_config_p config, const char* key,
  double* values) {
  config_p joint_config_a = (config_p)&config->joints;
  int i;

  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i)
    values[i] = deg_to_rad(config_get_float(&joint_config_a[i], key));

  return values;
}
