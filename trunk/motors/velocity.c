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

#include "velocity.h"

const char* era_motors_velocity_errors[] = {
  "success",
  "error starting velocity control operation",
  "error setting velocity state",
};

int era_motors_velocity_start(era_motors_p motors) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;
  epos_velocity_t velocity;

  epos_velocity_init(&velocity, 0.0);

  int result = EPOS_ERROR_NONE;
  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    result |= epos_velocity_start(&node_a[i], &velocity);

  if (result)
    return ERA_MOTORS_VELOCITY_ERROR_START;
  else
    return ERA_MOTORS_VELOCITY_ERROR_NONE;
}

void era_motors_velocity_stop(era_motors_p motors) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i)
    epos_control_stop(&node_a[i].control);
}

int era_motors_velocity_set_state(era_motors_p motors, era_velocity_state_p 
  vel_state) {
  int i;
  epos_node_p node_a = (epos_node_p)motors;
  double* omega_a = (double*)vel_state;
  epos_velocity_t velocity;

  int result = EPOS_ERROR_NONE;
  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i) {
    epos_velocity_init(&velocity, omega_a[i]);
    result |= epos_velocity_update(&node_a[i], &velocity);
  }

  if (result)
    return ERA_MOTORS_VELOCITY_ERROR_SET;
  else
    return ERA_MOTORS_VELOCITY_ERROR_NONE;
}
