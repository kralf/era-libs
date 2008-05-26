/*      Utility program for ERA limit determination
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <limits.h>

#include <controller.h>
#include <motors.h>

#define limit(x) x<0?1000000:-1000000

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s DEV\n", argv[0]);
    return -1;
  }

  era_init(argv[1]);

  era_motor_velocity_t motor_homing_velocity;
  era_arm_to_motor(0, &era_homing_velocity, 0, &motor_homing_velocity);

  era_motor_configuration_t limit_configuration = {
    .shoulder_yaw = limit(motor_homing_velocity.shoulder_yaw),
    .shoulder_roll = limit(motor_homing_velocity.shoulder_roll),
    .shoulder_pitch = limit(motor_homing_velocity.shoulder_pitch),
    .ellbow_pitch = limit(motor_homing_velocity.ellbow_pitch),
    .tool_roll = limit(motor_homing_velocity.tool_roll),
    .tool_opening = limit(motor_homing_velocity.tool_opening),
  };

  era_motors_set_configuration(&limit_configuration, &motor_homing_velocity);

  era_motors_wait(ERA_MOTORS_WAIT_FAULT);

  era_print_configuration(stdout);

  era_close();
  return 0;
}
