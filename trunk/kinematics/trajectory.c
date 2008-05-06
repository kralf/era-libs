/*	Trayectory Generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>

#include"trajectory.h"

static era_arm_velocity_t era_arm_velocity_max = {
  .shoulder_yaw = M_PI*13/36,
  .shoulder_roll = M_PI*2/5,
  .shoulder_pitch = M_PI*5/12,
  .ellbow_pitch = M_PI*5/12,
  .tool_roll = M_PI,
  .tool_opening = 0,
};

void era_get_velocity_limits(
  era_arm_velocity_t* arm_velocity_max) {
  memcpy(arm_velocity_max, &era_arm_velocity_max, sizeof(era_arm_velocity_t));
}

int era_test_velocity_limits(
  era_arm_velocity_t* arm_velocity) {
  int i;
  double* vel = (double*)arm_velocity;
  double* vel_max = (double*)&era_arm_velocity_max;

  for (i = 0; i <= sizeof(era_arm_velocity_t)/sizeof(double); i++)
    if (vel[i] > vel_max[i] || vel[i] < -vel_max[i])
    return 1;

  return 0;
}

int era_read_trajectory(
  const char *filename,
  era_tool_configuration_t** tool_configurations,
  double** timestamps) {
  FILE *fd;
  char buffer[255];
  era_tool_configuration_t point;
  double timestamp;
  int result = 0, points = 0, comments = 0;
  int i;

  fd = fopen(filename,"r");

  if (fd == NULL) {
    printf("File not found!\n");
    return 0;
  }

  while (fgets(buffer, sizeof(buffer), fd) != NULL) {
    if (buffer[0] == '/' && buffer[1] == '/') {
      comments++;
      continue;
    }

    result = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf",
      &point.x, &point.y, &point.z, &point.yaw, &point.roll, &point.opening,
      &timestamp);

    if (result != 6) {
      printf("Error in trajectory file at line %d!\n", (points+1)+comments);
      return 1;
    }

    *tool_configurations = realloc(*tool_configurations,
      (points+1)*sizeof(era_tool_configuration_t));
    memcpy(tool_configurations[points], &point,
      sizeof(era_tool_configuration_t));
    *timestamps = realloc(*timestamps, (points+1)*sizeof(double));
    *timestamps[points] = timestamp;

    points++;
  }

  return points;
}

int era_trajectory_velocities(
  era_tool_configuration_t* tool_configurations,
  double* timestamps,
  int num_tool_configurations,
  double dt,
  era_arm_velocity_t** arm_velocities,
  era_trajectory_error_t** errors) {
  era_tool_configuration_t m[num_tool_configurations];

  era_trajectory_mci_gradients(tool_configurations, num_tool_configurations, m);

  return era_trajectory_mci(tool_configurations, timestamps, m,
    num_tool_configurations, dt, arm_velocities, errors);
}

void era_trajectory_mci_gradients(
  era_tool_configuration_t* tool_configurations,
  int num_tool_configurations,
  era_tool_configuration_t* tool_configuration_gradients) {
  int i, j;
  double delta, alpha, beta, tau;

  double** tool = (double**)tool_configurations;
  double** m = (double**)tool_configuration_gradients;

  for (j = 0; j < sizeof(era_tool_configuration_t)/sizeof(double); j++) {
    m[0][j] = 0;
    m[num_tool_configurations-1][j] = 0;

    for(i = 1; i < num_tool_configurations-1; i++)
      m[i][j] = (tool[i+1][j]-tool[i-1][j])/2;

    for(i = 0; i < num_tool_configurations-1; i++) {
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
  era_tool_configuration_t* tool_configurations,
  double* timestamps,
  era_tool_configuration_t* tool_configuration_gradients,
  int num_tool_configurations,
  double dt,
  era_arm_velocity_t** arm_velocities,
  era_trajectory_error_t** errors) {
  int i, j, k, steps;
  double dj;
  int num_arm_velocities = 0;

  double** tool = (double**)tool_configurations;
  double** m = (double**)tool_configuration_gradients;
  double** arm;

  era_tool_configuration_t tool_configuration_a;
  era_tool_configuration_t tool_configuration_b;
  era_arm_configuration_t arm_configuration_a;
  era_arm_configuration_t arm_configuration_b;
  double* tool_a = (double*)&tool_configuration_a;
  double* tool_b = (double*)&tool_configuration_b;
  double* arm_a = (double*)&arm_configuration_a;
  double* arm_b = (double*)&arm_configuration_b;

  for (i = 0; i < (num_tool_configurations-1); i++) {
    steps = (int)(timestamps[i]/dt)+1;
    dj = 1/(double)steps;

    *arm_velocities = realloc(*arm_velocities,
      (num_arm_velocities+steps)*sizeof(era_arm_velocity_t));
    arm = (double**)arm_velocities;
    *errors = realloc(*errors,
      (num_arm_velocities+steps)*sizeof(era_trajectory_error_t));

    for (j = 0; j < steps; j++) {
      for (k = 0; k < 6; k++) {
        tool_b[k] = era_trajectory_eval(tool[i][k], tool[i+1][k], m[i][k],
          m[i+1][k], (j+1)*dj);
        tool_a[k] = era_trajectory_eval(tool[i][k], tool[i+1][k], m[i][k],
          m[i+1][k], j*dj);
      }

      errors[num_arm_velocities]->point = i;
      if (era_inverse_kinematics(&tool_configuration_b, &arm_configuration_b))
        errors[num_arm_velocities]->limits_exceeded = 1;
      else
        errors[num_arm_velocities]->limits_exceeded = 0;


      for (k = 0; k < 6; k++)
        arm[num_arm_velocities][k] = (arm_b[k]-arm_a[k])/dt;

      errors[num_arm_velocities]->velocity_exceeded =
        era_test_velocity_limits(arm_velocities[num_arm_velocities]);

      num_arm_velocities++;
    }
  }

  return num_arm_velocities;
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
