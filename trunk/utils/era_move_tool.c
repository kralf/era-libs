/*      Utility program for generating ERA position commands from
 *      tool configuration space coordinates
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <math.h>

#include <era.h>

int main(int argc, char **argv) {
  if (argc != 8) {
    fprintf(stderr, "Usage: %s DEV X Y Z YAW ROLL OPENING\n", argv[0]);
    return -1;
  }

  era_motors_init(argv[1]);

  era_tool_configuration_t target;

  sscanf(argv[2], "%lf", &target.x);
  sscanf(argv[3], "%lf", &target.y);
  sscanf(argv[4], "%lf", &target.z);
  sscanf(argv[5], "%lf", &target.yaw);
  sscanf(argv[6], "%lf", &target.roll);
  sscanf(argv[7], "%lf", &target.opening);

  target.yaw *= M_PI/180.0;
  target.roll *= M_PI/180.0;
  target.opening *= M_PI/180.0;

  int result = era_move_tool(&target, ERA_DEFAULT_VELOCITY, 1);

  if (result) era_print_error(stdout, result);

  era_motors_close();
  return 0;
}