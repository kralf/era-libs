/*      Utility program for generating ERA position commands from
 *      arm configuration space coordinates
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <math.h>

#include <controller.h>

int main(int argc, char **argv) {
  if (argc != 8) {
    fprintf(stderr, "Usage: %s DEV %s %s %s %s %s %s\n", argv[0],
      "SHOULDER_YAW", "SHOULDER_ROLL", "SHOULDER_PITCH",
      "ELLBOW_PITCH", "TOOL_ROLL", "TOOL_OPENING");
    return -1;
  }

  era_motors_init(argv[1]);

  era_arm_configuration_t target;

  sscanf(argv[2], "%lf", &target.shoulder_yaw);
  sscanf(argv[3], "%lf", &target.shoulder_roll);
  sscanf(argv[4], "%lf", &target.shoulder_pitch);
  sscanf(argv[5], "%lf", &target.ellbow_pitch);
  sscanf(argv[6], "%lf", &target.tool_roll);
  sscanf(argv[7], "%lf", &target.tool_opening);

  target.shoulder_yaw *= M_PI/180.0;
  target.shoulder_roll *= M_PI/180.0;
  target.shoulder_pitch *= M_PI/180.0;
  target.ellbow_pitch *= M_PI/180.0;
  target.tool_roll *= M_PI/180.0;
  target.tool_opening *= M_PI/180.0;

  int result = era_move(&target, 0.2, 1);

  if (result) era_print_error(stdout, result);

  era_motors_close();
  return 0;
}
