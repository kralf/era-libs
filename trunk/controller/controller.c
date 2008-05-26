/*      BlueBotics ERA-5/1 controller
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     9.5.2008
 */

#include <math.h>

#include "controller.h"
#include "motors.h"
#include "errors.h"

#define sqr(x) x*x

era_arm_configuration_t era_home = {
  .shoulder_yaw = 0.0*M_PI/180.0,
  .shoulder_roll = 10.0*M_PI/180.0,
  .shoulder_pitch = 0.0*M_PI/180.0,
  .ellbow_pitch = 25.0*M_PI/180.0,
  .tool_roll = -10.0*M_PI/180.0,
  .tool_opening = 0.0*M_PI/180.0,
};

const era_arm_velocity_t era_homing_velocity = {
  .shoulder_yaw = 3.0*M_PI/180.0,
  .shoulder_roll = -3.0*M_PI/180.0,
  .shoulder_pitch = -3.0*M_PI/180.0,
  .ellbow_pitch = -3.0*M_PI/180.0,
  .tool_roll = -15.0*M_PI/180.0,
  .tool_opening = -10.0*M_PI/180.0,
};

void era_print_configuration(
  FILE* stream) {
  int i;
  era_arm_configuration_t arm_configuration;
  era_arm_velocity_t arm_velocity;
  era_tool_configuration_t tool_configuration;

  era_get_configuration(&arm_configuration, &arm_velocity);
  era_forward_kinematics(&arm_configuration, &tool_configuration);

  fprintf(stream, "%s %21s  %27s  %18s\n",
    "ARM", "CONFIGURATION",
    "VELOCITY",
    "TOOL CONFIGURATION");

  fprintf(stream, "%14s: % 7.2f °  %14s: % 7.2f °/s  %7s: % 7.2f m\n",
    "shoulder_yaw", arm_configuration.shoulder_yaw*180/M_PI,
    "shoulder_yaw", arm_velocity.shoulder_yaw*180/M_PI,
    "x", tool_configuration.x);
  fprintf(stream, "%14s: % 7.2f °  %14s: % 7.2f °/s  %7s: % 7.2f m\n",
    "shoulder_roll", arm_configuration.shoulder_roll*180/M_PI,
    "shoulder_roll", arm_velocity.shoulder_roll*180/M_PI,
    "y", tool_configuration.y);
  fprintf(stream, "%14s: % 7.2f °  %14s: % 7.2f °/s  %7s: % 7.2f m\n",
    "shoulder_pitch", arm_configuration.shoulder_pitch*180/M_PI,
    "shoulder_pitch", arm_velocity.shoulder_pitch*180/M_PI,
    "z", tool_configuration.z);
  fprintf(stream, "%14s: % 7.2f °  %14s: % 7.2f °/s  %7s: % 7.2f °\n",
    "ellbow_pitch", arm_configuration.ellbow_pitch*180/M_PI,
    "ellbow_pitch", arm_velocity.ellbow_pitch*180/M_PI,
    "yaw", tool_configuration.yaw*180/M_PI);
  fprintf(stream, "%14s: % 7.2f °  %14s: % 7.2f °/s  %7s: % 7.2f °\n",
    "tool_roll", arm_configuration.tool_roll*180/M_PI,
    "tool_roll", arm_velocity.tool_roll*180/M_PI,
    "roll", tool_configuration.roll*180/M_PI);
  fprintf(stream, "%14s: % 7.2f °  %14s: % 7.2f °/s  %7s: % 7.2f °\n",
    "tool_opening", arm_configuration.tool_opening*180/M_PI,
    "tool_opening", arm_velocity.tool_opening*180/M_PI,
    "opening", tool_configuration.opening*180/M_PI);
}

int era_init(
  const char* dev) {
  int i;

  era_motors_init(dev);

  era_arm_configuration_t arm_limit;
  double* lim = (double*)&arm_limit;
  double* min = (double*)&era_arm_configuration_min;
  double* max = (double*)&era_arm_configuration_max;
  double* vel = (double*)&era_homing_velocity;

  for (i = 0; i < sizeof(era_arm_configuration_t)/sizeof(double); i++) {
    if (vel[i] < 0.0) lim[i] = min[i];
    else lim[i] = max[i];
  }

  era_motor_configuration_t motor_limit;
  era_motor_configuration_t motor_home;
  era_motor_velocity_t motor_homing_velocity;
  era_arm_to_motor(&arm_limit, &era_homing_velocity, &motor_limit,
    &motor_homing_velocity);
  era_arm_to_motor(&era_home, 0, &motor_home, 0);

  int result = era_motors_home(&motor_homing_velocity, &motor_limit,
    &motor_home);

  if (!result) era_motors_wait(ERA_MOTORS_WAIT_HOME_ATTAINED);

  return result;
}

void era_close() {
  era_motors_close();
}

void era_get_configuration(
  era_arm_configuration_t* configuration,
  era_arm_velocity_t* velocity) {
  era_motor_configuration_t motor_configuration;
  era_motor_velocity_t motor_velocity;

  if (configuration)
    era_motors_get_configuration(&motor_configuration, 0);
  if (velocity)
    era_motors_get_configuration(0, &motor_velocity);

  era_motor_to_arm(&motor_configuration, &motor_velocity,
    configuration, velocity);
}

int era_set_configuration(
  const era_arm_configuration_t* configuration,
  const era_arm_velocity_t* velocity) {
  int result;

  result = era_test_arm_configuration_limits(configuration);
  if (!result) result = era_test_arm_velocity_limits(velocity);

  if (!result) {
    era_motor_configuration_t motor_configuration;
    era_motor_velocity_t motor_velocity;
    era_arm_to_motor(configuration, velocity, &motor_configuration,
      &motor_velocity);

    if (configuration)
      result = era_motors_set_configuration(&motor_configuration,
      &motor_velocity);
    else
      result = era_motors_set_configuration(0, &motor_velocity);
  }

  return result;
}

int era_move(
  const era_arm_configuration_t* target,
  double velocity,
  int wait) {
  int result;

  era_arm_configuration_t current;
  era_arm_velocity_t arm_velocity;

  era_get_configuration(&current, 0);
  era_sync_arm_velocity(&current, target, velocity, &arm_velocity);

  result = era_set_configuration(target, &arm_velocity);

  if (!result && wait) era_motors_wait(ERA_MOTORS_WAIT_TARGET_REACHED);

  return result;
}

int era_move_home(
  double velocity,
  int wait) {
  return era_move(&era_home, velocity, wait);
}

int era_move_tool(
  const era_tool_configuration_t* target,
  double velocity,
  int wait) {
  era_arm_configuration_t arm_configuration;

  era_inverse_kinematics(target, &arm_configuration);

  return era_move(&arm_configuration, velocity, wait);
}

int era_move_trajectory(
  const era_arm_configuration_t* targets,
  const double* timestamps) {
  return 0;
}

int era_move_tool_trajectory(
  const era_tool_configuration_t* targets,
  const double* timestamps) {
  return 0;
}

double era_get_configuration_error(
  const era_arm_configuration_t* target) {
  int i;
  double squared_error = 0.0;

  era_arm_configuration_t current;
  double* current_conf = (double*)&current;
  double* target_conf = (double*)target;

  era_get_configuration(&current, 0);

  for (i = 0; i < sizeof(era_arm_configuration_t)/sizeof(double); i++)
    squared_error += sqr(target_conf[i]-current_conf[i]);

  return sqrt(squared_error);
}
