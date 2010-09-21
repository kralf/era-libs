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

#include <tulibs/timer.h>

#include "motors/velocity.h"

#include "control/closed_loop.h"

const char* era_control_closed_loop_errors[] = {
  "success",
  "error starting closed-loop control thread",
  "closed-loop control limit error",
};

param_t era_cl_default_shoulder_yaw_joint_params[] = {
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_P, "8.6496"},
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_I, "934.4593"},
};

param_t era_cl_default_shoulder_roll_joint_params[] = {
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_P, "11.8102"},
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_I, "908.65"},
};

param_t era_cl_default_shoulder_pitch_joint_params[] = {
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_P, "11.3374"},
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_I, "945.9724"},
};

param_t era_cl_default_elbow_pitch_joint_params[] = {
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_P, "9.8846"},
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_I, "878.7744"},
};

param_t era_cl_default_tool_roll_joint_params[] = {
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_P, "10.9886"},
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_I, "807.6947"},
};

param_t era_cl_default_tool_opening_joint_params[] = {
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_P, "12.5309"},
  {ERA_CONTROL_CLOSED_LOOP_PARAMETER_I, "767.6467"},
};

era_config_t era_control_closed_loop_default_config = {
  { 0,
    0 },
  { 
    { era_cl_default_shoulder_yaw_joint_params,
      sizeof(era_cl_default_shoulder_yaw_joint_params)/sizeof(param_t) },
    { era_cl_default_shoulder_roll_joint_params,
      sizeof(era_cl_default_shoulder_roll_joint_params)/sizeof(param_t) },
    { era_cl_default_shoulder_pitch_joint_params,
      sizeof(era_cl_default_shoulder_pitch_joint_params)/sizeof(param_t) },
    { era_cl_default_elbow_pitch_joint_params,
      sizeof(era_cl_default_elbow_pitch_joint_params)/sizeof(param_t) },
    { era_cl_default_tool_roll_joint_params,
      sizeof(era_cl_default_tool_roll_joint_params)/sizeof(param_t) },
    { era_cl_default_tool_opening_joint_params,
      sizeof(era_cl_default_tool_opening_joint_params)/sizeof(param_t) },
  },
};

void* era_control_closed_loop_run(void* arg);
void era_control_closed_loop_cleanup(void* arg);

int era_control_closed_loop_start(thread_p thread, era_arm_p arm, 
  thread_mutex_p mutex, era_trajectory_p trajectory, double frequency) {
//   if (era_dynamics_limits_test_velocity_profile(&arm->dyn_limits, profile))
//     return ERA_CONTROL_CLOSED_LOOP_ERROR_LIMITS;

  era_control_closed_loop_arg_p control_arg = 
    malloc(sizeof(era_control_closed_loop_arg_t));

  control_arg->arm = arm;
  control_arg->mutex = mutex;

  control_arg->trajectory = trajectory;

  era_config_joint_get_float(&era_control_closed_loop_default_config,
    ERA_CONTROL_CLOSED_LOOP_PARAMETER_P, (double*)&control_arg->p);
  era_config_joint_get_float(&era_control_closed_loop_default_config,
    ERA_CONTROL_CLOSED_LOOP_PARAMETER_I, (double*)&control_arg->i);

  control_arg->seg_index = 0;
  control_arg->start_time = 0.0;

  era_joint_init_state(&control_arg->error);
  era_velocity_init_state(&control_arg->output);
  control_arg->timestamp = 0.0;

  era_joint_state_t joint_state;
  era_trajectory_evaluate(control_arg->trajectory, 0.0, 0, &joint_state, 0, 0);

  thread_mutex_lock(control_arg->mutex);
  if (!era_move_joints(control_arg->arm, &joint_state, 1.0) &&
    !era_motors_velocity_start(&control_arg->arm->motors) &&
    !thread_start(thread, era_control_closed_loop_run, 
      era_control_closed_loop_cleanup, control_arg, frequency)) {
    thread_mutex_unlock(control_arg->mutex);
    return ERA_CONTROL_CLOSED_LOOP_ERROR_NONE;
  }
  else {
    thread_mutex_unlock(control_arg->mutex);
    return ERA_CONTROL_CLOSED_LOOP_ERROR_START;
  }
}

void era_control_closed_loop_exit(thread_p thread) {
  thread_exit(thread, 1);
}

void* era_control_closed_loop_run(void* arg) {
  era_joint_state_t act_joint_state, dem_joint_state;
  era_velocity_state_t dem_vel_state;
  era_control_closed_loop_arg_p control_arg = arg;

  double* act_joint_state_a = (double*)&act_joint_state;
  double* dem_joint_state_a = (double*)&dem_joint_state;
  double* dem_vel_state_a = (double*)&dem_vel_state;

  double* p_a = (double*)&control_arg->p;
  double* i_a = (double*)&control_arg->i;

  double* error_a = (double*)&control_arg->error;
  double* output_a = (double*)&control_arg->output;

  double timestamp = control_arg->timestamp;
  thread_mutex_lock(control_arg->mutex);
  timer_start(&control_arg->timestamp);
  era_get_joint_state(control_arg->arm, &act_joint_state);
  timer_correct(&control_arg->timestamp);

  double time = 0.0;
  if (control_arg->start_time == 0.0)
    timer_start(&control_arg->start_time);
  else
    time = timer_stop(control_arg->start_time);

  control_arg->seg_index = era_trajectory_evaluate(control_arg->trajectory, 
    time, control_arg->seg_index, &dem_joint_state, &dem_vel_state, 0);
  if (control_arg->seg_index < 0) {
    thread_mutex_unlock(control_arg->mutex);
    thread_self_exit();
  
    return 0;
  }

  int i;
  double t_s = (timestamp > 0.0) ? control_arg->timestamp-timestamp : 0.0;
  for (i = 0; i < sizeof(era_joint_state_t)/sizeof(double); ++i) {
    double e = dem_joint_state_a[i]-act_joint_state_a[i];
    double a = p_a[i]*(0.5*t_s/i_a[i]+1.0);
    double b = p_a[i]*(0.5*t_s/i_a[i]-1.0);

    double u = a*e+b*error_a[i]+output_a[i];
    
    dem_vel_state_a[i] += u;

    error_a[i] = e;
    output_a[i] = u;
  }

  era_motors_velocity_set_state(&control_arg->arm->motors, &dem_vel_state);

  thread_mutex_unlock(control_arg->mutex);

  return 0;
}

void era_control_closed_loop_cleanup(void* arg) {
  era_control_closed_loop_arg_p control_arg = arg;

  thread_mutex_lock(control_arg->mutex);
  era_motors_velocity_stop(&control_arg->arm->motors);
  thread_mutex_unlock(control_arg->mutex);

  free(control_arg);
}
