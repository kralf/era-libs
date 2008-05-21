/*      BlueBotics ERA-5/1 controller
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     9.5.2008
 */

#include "controller.h"

void era_print_configuration(
  FILE* stream) {
  era_arm_configuration_t arm;

  era_position_mode_get(&arm);
  era_print_arm_configuration(stream, &arm);
}

void era_init(
  const char* dev,
  int homing_mode) {
  era_motors_init(dev);
  era_motors_home(homing_mode);

  if (homing_mode != ERA_HOMING_MODE_NONE)
    era_motors_wait(ERA_HOME_ATTAINED);
}

void era_close() {
  era_motors_close();
}

void era_move_home(
  double velocity,
  int wait) {
  era_arm_configuration_t arm_configuration;

  era_motor_to_arm(&era_motor_home, 0, &arm_configuration, 0);

  era_move(&arm_configuration, velocity, wait);
}

void era_move(
  const era_arm_configuration_t* target,
  double velocity,
  int wait) {
  era_arm_configuration_t current;
  era_arm_velocity_t arm_velocity;

  era_position_mode_get(&current);
  era_sync_velocity(&current, target, velocity, &arm_velocity);

  era_motors_set_mode(ERA_OPERATION_MODE_POSITION);
  era_position_mode_set(target, &arm_velocity);

  if (wait) era_motors_wait(ERA_TARGET_REACHED);
}

void era_move_tool(
  const era_tool_configuration_t* target,
  double velocity,
  int wait) {
  era_arm_configuration_t arm_configuration;

  era_inverse_kinematics(target, &arm_configuration);

  era_move(&arm_configuration, velocity, wait);
}

void era_move_trajectory(
  const era_arm_configuration_t* targets,
  const double* timestamps) {
}

void era_move_tool_trajectory(
  const era_tool_configuration_t* targets,
  const double* timestamps) {
}
