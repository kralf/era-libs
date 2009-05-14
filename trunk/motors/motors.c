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

#include <security/security.h>

#include "motors.h"

const char* era_motors_errors[] = {
  "success",
  "error opening motors",
  "error closing motors",
  "error homing motors",
  "error homing motors",
  "wait operation timed out",
};

void era_motors_init(era_motors_p motors, can_device_p can_dev, 
  era_config_joint_p config) {
  if (!can_dev) {
    can_dev = malloc(sizeof(can_device_t));
    can_init(can_dev, 0);
  }

  epos_node_p node_a = (epos_node_p)motors;
  config_p config_a = (config_p)config;
  int i;
  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    epos_init(&node_a[i], can_dev, &config_a[i]);
}

void era_motors_destroy(era_motors_p motors) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    epos_destroy(&node_a[i]);
}

int era_motors_open(era_motors_p motors) {
  int i, result = EPOS_ERROR_NONE;
  epos_node_p node_a = (epos_node_p)motors;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    result &= epos_open(&node_a[i]);

  if (result)
    return ERA_MOTORS_ERROR_OPEN;
  else
    return ERA_MOTORS_ERROR_NONE;
}

int era_motors_close(era_motors_p motors) {
  int i, result = EPOS_ERROR_NONE;
  epos_node_p node_a = (epos_node_p)motors;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    result &= epos_close(&node_a[i]);

  if (result)
    return ERA_MOTORS_ERROR_CLOSE;
  else
    return ERA_MOTORS_ERROR_NONE;
}

int era_motors_wait_status(era_motors_p motors, short status, double timeout) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;

  struct timeval tv;
  gettimeofday(&tv, 0);
  double time = tv.tv_sec+tv.tv_usec/1e6;

  short result;
  do {
    result = status;
//     for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    for (i = 0; i < 1; ++i)
      result &= (status & epos_device_get_status(&node_a[i].dev));

    if (timeout >= 0.0) {
      gettimeofday(&tv, 0);
      if (tv.tv_sec+tv.tv_usec/1e6-time > timeout)
        return ERA_MOTORS_ERROR_WAIT_TIMEOUT;
    }
  }
  while (!result);

  return ERA_MOTORS_ERROR_NONE;
}

int era_motors_home(era_motors_p motors, era_security_p security, 
  double timeout) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;

  int result = EPOS_ERROR_NONE;
//   for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
  for (i = 0; i < 1; ++i)
    result &= epos_home(&node_a[i], 0.0);

  if (!result) {
    era_motors_home_wait(motors, timeout);
    return ERA_MOTORS_ERROR_NONE;
  }
  else {
    era_motors_home_stop(motors);
    return ERA_MOTORS_ERROR_HOME;
  }
}

void era_motors_home_stop(era_motors_p motors) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;

//   for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
  for (i = 0; i < 1; ++i)
    epos_home_stop(&node_a[i]);
}

int era_motors_home_wait(era_motors_p motors, double timeout) {
  return era_motors_wait_status(motors, EPOS_HOME_STATUS_REACHED, timeout);
}
