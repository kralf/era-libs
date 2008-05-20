/*      Utility program for ERA path tracking
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <controller.h>

int main(int argc, char **argv) {
  if (argc < 3) {
    fprintf(stderr, "Usage: %s DEV PATH_FILE\n", argv[0]);
    return -1;
  }

  era_init(argv[1], ERA_HOMING_MODE_NONE);

  era_tool_configuration_t* tool_configurations;
  double* timestamps;

  if (era_read_trajectory(argv[2], &tool_configurations, &timestamps)) {
    free(tool_configurations);
    free(timestamps);
  }

  era_close();
  return 0;
}
