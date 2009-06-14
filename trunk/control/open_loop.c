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

#include <timer.h>

#include <motors/velocity.h>

#include "open_loop.h"

const char* era_control_open_loop_errors[] = {
  "success",
  "error starting open-loop control thread",
  "open-loop control limit error",
};

void* era_control_open_loop_run(void* arg);
void era_control_open_loop_cleanup(void* arg);

int era_control_open_loop_start(thread_p thread, era_arm_p arm, thread_mutex_p
  mutex, era_trajectory_p trajectory, double frequency) {
//   if (era_dynamics_limits_test_velocity_profile(&arm->dyn_limits, profile))
//     return ERA_CONTROL_OPEN_LOOP_ERROR_LIMITS;

  era_control_open_loop_arg_p control_arg = 
    malloc(sizeof(era_control_open_loop_arg_t));

  control_arg->arm = arm;
  control_arg->mutex = mutex;

  control_arg->trajectory = trajectory;

  control_arg->seg_index = 0;
  control_arg->start_time = 0.0;

  era_joint_state_t joint_state;
  era_trajectory_evaluate(control_arg->trajectory, 0.0, 0, &joint_state, 0, 0);

  thread_mutex_lock(control_arg->mutex);
  if (!era_move_joints(control_arg->arm, &joint_state, 1.0) &&
    !era_motors_velocity_start(&control_arg->arm->motors) &&
    !thread_start(thread, era_control_open_loop_run, 
      era_control_open_loop_cleanup, control_arg, frequency)) {
    thread_mutex_unlock(control_arg->mutex);
    return ERA_CONTROL_OPEN_LOOP_ERROR_NONE;
  }
  else {
    thread_mutex_unlock(control_arg->mutex);
    return ERA_CONTROL_OPEN_LOOP_ERROR_START;
  }
}

void era_control_open_loop_exit(thread_p thread) {
  thread_exit(thread, 1);
}

void* era_control_open_loop_run(void* arg) {
  double time = 0.0;
  era_joint_state_t dem_joint_state;
  era_velocity_state_t dem_vel_state;
  era_control_open_loop_arg_p control_arg = arg;

  if (control_arg->start_time == 0.0)
    timer_start(&control_arg->start_time);
  else
    time = timer_stop(control_arg->start_time);

  control_arg->seg_index = era_trajectory_evaluate(control_arg->trajectory, 
    time, control_arg->seg_index, &dem_joint_state, &dem_vel_state, 0);
  if (control_arg->seg_index < 0) {
    thread_self_exit();
    return 0;
  }

  thread_mutex_lock(control_arg->mutex);

  era_motors_velocity_set_state(&control_arg->arm->motors, &dem_vel_state);

  thread_mutex_unlock(control_arg->mutex);

  return 0;
}

void era_control_open_loop_cleanup(void* arg) {
  era_control_open_loop_arg_p control_arg = arg;

  thread_mutex_lock(control_arg->mutex);
  era_motors_velocity_stop(&control_arg->arm->motors);
  thread_mutex_unlock(control_arg->mutex);

  free(control_arg);
}
