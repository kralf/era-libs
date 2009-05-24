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

#include <base/era.h>
#include <security/security.h>

#include "home.h"

const char* era_motors_home_errors[] = {
  "success",
  "error starting motor homing",
};

void era_motors_home_init(era_motors_p motors) {
  epos_node_p node_a = (epos_node_p)motors;
  int i;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i) {
    epos_home_method_t method = config_get_int(&node_a[i].config,
      EPOS_PARAMETER_HOME_METHOD);
    double gear_trans = config_get_float(&node_a[i].config,
      EPOS_PARAMETER_GEAR_TRANSMISSION);
    double home_pos = config_get_float(&node_a[i].config, 
      EPOS_PARAMETER_HOME_POSITION);

    double min_pos = config_get_float(&node_a[i].config, 
      ERA_PARAMETER_JOINT_MIN_POSITION);
    double max_pos = config_get_float(&node_a[i].config, 
      ERA_PARAMETER_JOINT_MAX_POSITION);
    double limit_pos;

    if ((method == epos_home_neg_switch_index) ||
      (method == epos_home_neg_switch) ||
      (method == epos_home_neg_current_index) ||
      (method == epos_home_neg_current))
      limit_pos = (gear_trans > 0.0) ? min_pos : max_pos;
    else
      limit_pos = (gear_trans > 0.0) ? max_pos : min_pos;

    config_set_float(&node_a[i].config, EPOS_PARAMETER_HOME_OFFSET,
      abs(limit_pos-home_pos));
  }
}

int era_motors_home_start(era_motors_p motors, era_security_p security) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;

  int result = EPOS_ERROR_NONE;
  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    result &= epos_home(&node_a[i], 0.0);

  if (result) {
    era_motors_home_stop(motors);
    return ERA_MOTORS_HOME_ERROR_START;
  }
  else
    return ERA_MOTORS_HOME_ERROR_NONE;
}

int era_motors_home_wait(era_motors_p motors, double timeout) {
  return era_motors_wait_status(motors, EPOS_HOME_STATUS_REACHED, timeout);
}

void era_motors_home_stop(era_motors_p motors) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    epos_home_stop(&node_a[i]);
}
