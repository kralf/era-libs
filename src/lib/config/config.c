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
  sprintf(arm_prefix, "%s%s", (prefix) ? prefix : ERA_CONFIG_ARG_PREFIX, 
    ERA_CONFIG_ARG_PREFIX_ARM);

  config_init_arg(&config->arm, argc, argv, arm_prefix);

  config_p joint_config_a = (config_p)&config->joints;
  int i;
  for (i = 0; i < sizeof(era_config_joint_t)/sizeof(config_t); ++i) {
    char joint_prefix[256];
    sprintf(joint_prefix, "%s%s", (prefix) ? prefix : ERA_CONFIG_ARG_PREFIX, 
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
