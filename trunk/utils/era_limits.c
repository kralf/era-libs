/*      Utility program for ERA limit determination
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <limits.h>

#include <controller.h>
#include <motors.h>

#define limit(x) x<0?INT_MAX:INT_MIN

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s DEV\n", argv[0]);
    return -1;
  }

  //era_init(argv[1]);
  //era_motors_init(argv[1]);

//   const era_motor_configuration_t era_motor_home_offset = {
//     .shoulder_yaw = -45592,
//     .shoulder_roll = -42443,
//     .shoulder_pitch = -42722,
//     .ellbow_pitch = -35608,
//     .tool_roll = -182580,
//     .tool_opening = -36300,
//   };

//   era_motor_configuration_t limit_configuration = {
//     .shoulder_yaw = limit(era_homing_velocity.shoulder_yaw),
//     .shoulder_roll = limit(era_homing_velocity.shoulder_roll),
//     .shoulder_pitch = limit(era_homing_velocity.shoulder_pitch),
//     .ellbow_pitch = limit(era_homing_velocity.ellbow_pitch),
//     .tool_roll = limit(era_homing_velocity.tool_roll),
//     .tool_opening = limit(era_homing_velocity.tool_opening),
//   };

  //era_print_motor_configuration(stdout, &era_motor_home_offset);
  //era_motor_velocity_t motor_homing_velocity;
  //era_arm_to_motor(0, &era_homing_velocity, 0, &motor_homing_velocity);
/*  era_arm_configuration_t h;
  era_motor_to_arm(&era_motor_home_offset, 0, &h, 0);
  era_print_arm_configuration(stdout, &h);*/
//   era_motor_configuration_t k;
//   era_arm_to_motor(&h, 0, &k, 0);
//   era_print_motor_configuration(stdout, &k);

  //era_motors_set_configuration(&limit_configuration, &motor_homing_velocity);

  //era_motors_wait(ERA_MOTORS_WAIT_FAULT);

  //era_print_configuration(stdout);

  //era_close();
  return 0;
}
