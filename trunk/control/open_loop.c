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

#include <motors/velocity.h>

#include "open_loop.h"

const char* era_control_open_loop_errors[] = {
  "success",
  "error starting open-loop control thread",
  "open-loop control limit error",
};

void* era_control_open_loop_run(void* arg);

int era_control_open_loop_start(thread_p thread, era_arm_p arm, thread_mutex_p
  mutex, era_velocity_profile_p profile) {
  if (era_dynamics_limits_test_velocity_profile(&arm->dyn_limits, profile))
    return ERA_CONTROL_OPEN_LOOP_ERROR_LIMITS;

  era_control_open_loop_arg_p control_arg = 
    malloc(sizeof(era_control_open_loop_arg_t));

  control_arg->arm = arm;
  control_arg->mutex = mutex;

  control_arg->profile = profile;

  if (!thread_start(thread, era_control_open_loop_run, control_arg, 0.0))
    return ERA_CONTROL_OPEN_LOOP_ERROR_NONE;
  else
    return ERA_CONTROL_OPEN_LOOP_ERROR_START;
}

void era_control_open_loop_exit(thread_p thread) {
  thread_exit(thread, 1);
  free(thread->arg);
}

void* era_control_open_loop_run(void* arg) {
  int i;
  double start_time;

  era_control_open_loop_arg_p control_arg = arg;
  era_velocity_profile_p profile = control_arg->profile;
  timer_start(&start_time);

  thread_mutex_lock(control_arg->mutex);
  era_motors_velocity_start(&control_arg->arm->motors);
  thread_mutex_unlock(control_arg->mutex);

  for (i = 0; i < profile->num_points; i++) {
    thread_test_cancel();

    thread_mutex_lock(control_arg->mutex);
    era_motors_velocity_set_state(&control_arg->arm->motors, 
      &profile->points[i]);
    thread_mutex_unlock(control_arg->mutex);

    timer_wait(start_time, 1.0/profile->timestamps[i]);
  }

  thread_mutex_lock(control_arg->mutex);
  era_motors_velocity_stop(&control_arg->arm->motors);
  thread_mutex_unlock(control_arg->mutex);

  return 0;
}
