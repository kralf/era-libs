/*	Velocity calculations for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#include <stdlib.h>
#include <math.h>
#include <string.h>

#include "velocity.h"

#define max(x, y) x>=y?x:y

const era_arm_velocity_t era_arm_velocity_max = {
  .shoulder_yaw = M_PI*13/36,
  .shoulder_roll = M_PI*2/5,
  .shoulder_pitch = M_PI*5/12,
  .ellbow_pitch = M_PI*5/12,
  .tool_roll = M_PI,
  .tool_opening = M_PI,
};

void era_print_velocity(
  FILE* stream,
  const era_arm_velocity_t* arm_velocity) {
  fprintf(stream, "shoulder_yaw:   %f °/s\n",
    arm_velocity->shoulder_yaw*180/M_PI);
  fprintf(stream, "shoulder_roll:  %f °/s\n",
    arm_velocity->shoulder_roll*180/M_PI);
  fprintf(stream, "shoulder_pitch: %f °/s\n",
    arm_velocity->shoulder_pitch*180/M_PI);
  fprintf(stream, "ellbow_pitch:   %f °/s\n",
    arm_velocity->ellbow_pitch*180/M_PI);
  fprintf(stream, "tool_roll:      %f °/s\n",
    arm_velocity->tool_roll*180/M_PI);
  fprintf(stream, "tool_opening:   %f °/s\n",
    arm_velocity->tool_opening*180/M_PI);
}

int era_test_velocity_limits(
  const era_arm_velocity_t* arm_velocity) {
  int i;
  double* vel = (double*)arm_velocity;
  double* vel_max = (double*)&era_arm_velocity_max;

  for (i = 0; i < sizeof(era_arm_velocity_t)/sizeof(double); i++)
    if (vel[i] > vel_max[i] || vel[i] < -vel_max[i])
    return 1;

  return 0;
}

double era_sync_velocity(
  const era_arm_configuration_t* arm_start_configuration,
  const era_arm_configuration_t* arm_target_configuration,
  double velocity,
  era_arm_velocity_t* arm_velocity) {
  int i;
  double max_diff = 0.0, max_vel = 0.0;

  double* start = (double*)arm_start_configuration;
  double* target = (double*)arm_target_configuration;
  double* vel = (double*)arm_velocity;
  double* vel_max = (double*)&era_arm_velocity_max;

  for (i = 0; i < sizeof(era_arm_configuration_t)/sizeof(double); i++)
    max_diff = max(max_diff, fabs(target[i]-start[i]));

  if (max_diff == 0.0) {
    for (i = 0; i < sizeof(era_arm_configuration_t)/sizeof(double); i++)
      vel[i] = 0.0;

    return 0;
  }

  for (i = 0; i < sizeof(era_arm_configuration_t)/sizeof(double); i++) {
    vel[i] = (target[i]-start[i])/max_diff;
    max_vel = max(max_vel, fabs(vel[i])*velocity*vel_max[i]);
  }

  for (i = 0; i < sizeof(era_arm_configuration_t)/sizeof(double); i++)
    vel[i] *= max_vel;

  return max_diff/max_vel;
}
