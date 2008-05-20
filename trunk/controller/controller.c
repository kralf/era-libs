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

void era_move_home() {
  era_arm_configuration_t home;
  era_arm_velocity_t vel;

  era_motor_to_arm(&era_motor_home, &era_motor_homing_velocity, &home, &vel);

  era_motors_set_mode(ERA_OPERATION_MODE_POSITION);
  era_position_mode_set(&home, &vel);

  era_motors_wait(ERA_TARGET_REACHED);
}

void era_move_trajectory(
  era_tool_configuration_t* tool_configurations,
  double* timestamps) {
}
