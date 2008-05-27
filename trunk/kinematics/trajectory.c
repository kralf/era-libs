/*	Trayectory generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#include <stdlib.h>
#include <math.h>

#include "trajectory.h"
#include "errors.h"

void era_print_tool_trajectory(
  FILE* stream,
  const era_tool_configuration_t* tool_trajectory,
  int num_configurations) {
  int i;

  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s\n",
    "x",
    "y",
    "z",
    "yaw",
    "roll",
    "opening");

  for (i = 0; i < num_configurations; i++) {
    fprintf(stream,
      "%12.4f m  %12.4f m  %12.4f m  %12.2f °  %12.2f °  %12.2f °\n",
      tool_trajectory[i].x,
      tool_trajectory[i].y,
      tool_trajectory[i].z,
      tool_trajectory[i].yaw*180/M_PI,
      tool_trajectory[i].roll*180/M_PI,
      tool_trajectory[i].opening*180/M_PI);
  }
}

void era_print_arm_trajectory(
  FILE* stream,
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations) {
  int i;

  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s\n",
    "shoulder_yaw",
    "shoulder_roll",
    "shoulder_pitch",
    "ellbow_pitch",
    "tool_roll",
    "tool_opening");

  for (i = 0; i < num_configurations; i++) {
    fprintf(stream,
      "%12.2f °  %12.2f °  %12.2f °  %12.2f °  %12.2f °  %12.2f °\n",
      arm_trajectory[i].shoulder_yaw*180/M_PI,
      arm_trajectory[i].shoulder_roll*180/M_PI,
      arm_trajectory[i].shoulder_pitch*180/M_PI,
      arm_trajectory[i].ellbow_pitch*180/M_PI,
      arm_trajectory[i].tool_roll*180/M_PI,
      arm_trajectory[i].tool_opening*180/M_PI);
  }
}

int era_trajectory_test_arm_configuration_limits(
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations) {
  int i, result;

  for (i = 0; (i < num_configurations) && !result; ++i)
    result = era_test_arm_configuration_limits(&arm_trajectory[i]);

  return result;
}

void era_trajectory_forward_kinematics(
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations,
  era_tool_configuration_t* tool_trajectory) {
  int i;

  for (i = 0; i < num_configurations; i++)
    era_forward_kinematics(&arm_trajectory[i], &tool_trajectory[i]);
}

void era_trajectory_inverse_kinematics(
  const era_tool_configuration_t* tool_trajectory,
  int num_configurations,
  era_arm_configuration_t* arm_trajectory) {
  int i;

  for (i = 0; i < num_configurations; i++)
    era_inverse_kinematics(&tool_trajectory[i], &arm_trajectory[i]);
}

int era_trajectory_velocities(
  const era_tool_configuration_t* trajectory,
  const double* timestamps,
  int num_configurations,
  double dt,
  era_arm_velocity_t** velocity_profile,
  era_trajectory_error_t** errors) {
  era_tool_configuration_t m[num_configurations];

  era_trajectory_mci_gradients(trajectory, num_configurations, m);

  return era_trajectory_mci(trajectory, timestamps, m,
    num_configurations, dt, velocity_profile, errors);
}

void era_trajectory_mci_gradients(
  const era_tool_configuration_t* trajectory,
  int num_configurations,
  era_tool_configuration_t* trajectory_gradients) {
  int i, j;
  double delta, alpha, beta, tau;

  double** tool = (double**)trajectory;
  double** m = (double**)trajectory_gradients;

  for (j = 0; j < sizeof(era_tool_configuration_t)/sizeof(double); j++) {
    m[0][j] = 0;
    m[num_configurations-1][j] = 0;

    for(i = 1; i < num_configurations-1; i++)
      m[i][j] = (tool[i+1][j]-tool[i-1][j])/2;

    for(i = 0; i < num_configurations-1; i++) {
      delta = tool[i+1][j]-tool[i][j];

      if (delta == 0) {
        m[i][j] = 0;
        m[i+1][j] = 0;
      }
      else {
        alpha = m[i][j]/delta;
        beta = m[i+1][j]/delta;

        if (alpha+beta-2 > 0 &&
          2*alpha+beta-3 > 0 &&
          alpha+2*beta-3 > 0 &&
          alpha-(2*alpha+beta-3)*(2*alpha+beta-3)/(alpha+beta-2)/3 < 0) {
          tau = 3/sqrt(alpha*alpha + beta*beta);
          m[i][j] = tau*alpha*delta;
          m[i+1][j] = tau*beta*delta;
        }
      }
    }
  }
}

int era_trajectory_mci(
  const era_tool_configuration_t* trajectory,
  const double* timestamps,
  const era_tool_configuration_t* trajectory_gradients,
  int num_configurations,
  double dt,
  era_arm_velocity_t** velocity_profile,
  era_trajectory_error_t** errors) {
  int i, j, k, steps;
  double dj;
  int num_velocities = 0;

  double** tool = (double**)trajectory;
  double** m = (double**)trajectory_gradients;
  double** arm;

  era_tool_configuration_t tool_configuration_a;
  era_tool_configuration_t tool_configuration_b;
  era_arm_configuration_t arm_configuration_a;
  era_arm_configuration_t arm_configuration_b;
  double* tool_a = (double*)&tool_configuration_a;
  double* tool_b = (double*)&tool_configuration_b;
  double* arm_a = (double*)&arm_configuration_a;
  double* arm_b = (double*)&arm_configuration_b;

  for (i = 0; i < (num_configurations-1); i++) {
    steps = (int)(timestamps[i]/dt)+1;
    dj = 1/(double)steps;

    *velocity_profile = realloc(*velocity_profile,
      (num_velocities+steps)*sizeof(era_arm_velocity_t));
    arm = (double**)velocity_profile;
    *errors = realloc(*errors,
      (num_velocities+steps)*sizeof(era_trajectory_error_t));

    for (j = 0; j < steps; j++) {
      for (k = 0; k < 6; k++) {
        tool_b[k] = era_trajectory_eval(tool[i][k], tool[i+1][k], m[i][k],
          m[i+1][k], (j+1)*dj);
        tool_a[k] = era_trajectory_eval(tool[i][k], tool[i+1][k], m[i][k],
          m[i+1][k], j*dj);
      }

      era_inverse_kinematics(&tool_configuration_b, &arm_configuration_b);

      errors[num_velocities]->limits_exceeded =
        era_test_arm_configuration_limits(&arm_configuration_b);

      for (k = 0; k < 6; k++)
        arm[num_velocities][k] = (arm_b[k]-arm_a[k])/dt;

      errors[num_velocities]->velocity_exceeded =
        era_test_arm_velocity_limits(velocity_profile[num_velocities]);

      num_velocities++;
    }
  }

  return num_velocities;
}

double era_trajectory_eval(
  double p_a,
  double p_b,
  double m_a,
  double m_b,
  double t) {
  return (2*t*t*t-3*t*t+1)*p_a
    +(t*t*t-2*t*t+t)*m_a
    +(-2*t*t*t+3*t*t)*p_b
    +(t*t*t-t*t)*m_b;
}
