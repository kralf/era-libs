/*      BlueBotics ERA-5/1 controller
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     9.5.2008
 */

#include "controller.h"
#include "errors.h"

void era_print_configuration(
  FILE* stream) {
  era_arm_configuration_t arm;
  era_tool_configuration_t tool;

  era_position_mode_get(&arm);
  era_forward_kinematics(&arm, &tool);

  era_print_arm_configuration(stream, &arm);
  era_print_tool_configuration(stream, &tool);
}

void era_init(
  const char* dev) {
  era_motors_init(dev);

  era_arm_configuration_t arm_min, arm_max;

  era_motors_find_limits(&arm_min, &arm_max);
}

void era_close() {
  era_motors_close();
}

int era_home(
  double velocity,
  int wait) {
  era_arm_configuration_t arm_configuration;

  era_motor_to_arm(&era_motor_home, 0, &arm_configuration, 0);

  return era_move(&arm_configuration, velocity, wait);
}

int era_move(
  const era_arm_configuration_t* target,
  double velocity,
  int wait) {
  int result;

  era_arm_configuration_t current;
  era_arm_velocity_t arm_velocity;

  era_position_mode_get(&current);
  era_sync_velocity(&current, target, velocity, &arm_velocity);

  era_motors_set_mode(ERA_OPERATION_MODE_POSITION);
  result = era_position_mode_set(target, &arm_velocity);

  if (!result && wait) era_motors_wait(ERA_TARGET_REACHED);

  return result;
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
