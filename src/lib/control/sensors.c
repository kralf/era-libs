/***************************************************************************
 *   Copyright (C) 2008 by Fritz Stoeckli, Ralf Kaestner                   *
 *   stfritz@ethz.ch, ralf.kaestner@gmail.com                              *
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

#include "control/sensors.h"

const char* era_control_sensors_errors[] = {
  "success",
  "error starting sensor acquisition thread",
};

void* era_control_sensors_run(void* arg);

int era_control_sensors_start(thread_p thread, era_arm_p arm, thread_mutex_p
  mutex, era_control_sensors_handler_p handler, double frequency) {
  era_control_sensors_arg_p sensor_arg = 
    malloc(sizeof(era_control_sensors_arg_t));

  sensor_arg->arm = arm;
  sensor_arg->mutex = mutex;

  sensor_arg->handler = handler;
  sensor_arg->timestamp = 0.0;

  if (!thread_start(thread, era_control_sensors_run, 0, sensor_arg, frequency))
    return ERA_CONTROL_SENSORS_ERROR_NONE;
  else
    return ERA_CONTROL_SENSORS_ERROR_START;
}

void era_control_sensors_exit(thread_p thread) {
  thread_exit(thread, 1);
  free(thread->arg);
}

void* era_control_sensors_run(void* arg) {
  era_joint_state_t joint_state;
  era_velocity_state_t vel_state;

  era_control_sensors_arg_p sensor_arg = arg;
  double timestamp = sensor_arg->timestamp;

  timer_start(&sensor_arg->timestamp);
  thread_mutex_lock(sensor_arg->mutex);
  era_get_joint_state(sensor_arg->arm, &joint_state);
  era_get_velocity_state(sensor_arg->arm, &vel_state);
  thread_mutex_unlock(sensor_arg->mutex);
  timer_correct(&sensor_arg->timestamp);

  if (sensor_arg->handler)
    sensor_arg->handler(&joint_state, &vel_state, (timestamp > 0.0) ? 
      1.0/(sensor_arg->timestamp-timestamp) : 0.0);

  return 0;
}
