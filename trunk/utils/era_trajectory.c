/*      Utility program for ERA path tracking
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <controller.h>

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s DEV PATH_FILE\n", argv[0]);
    return -1;
  }

  era_motors_init(argv[1]);

  era_tool_configuration_t* tool_configurations;
  double* timestamps;

  int num_tool_configurations = era_read_trajectory(argv[2],
    &tool_configurations, &timestamps);

  if (num_tool_configurations) {
    //era_trajectory_velocities(tool_configurations, timestamps,
      //num_tool_configurations);

    //era_move_trajectory(tool_configurations, timestamps, 1);

    free(tool_configurations);
    free(timestamps);
  }

  era_motors_close();
  return 0;
}
