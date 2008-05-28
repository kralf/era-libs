/*      Utility program for generating ERA arm trajectories from
 *      tool trajectories
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <math.h>

#include <trajectory.h>
#include <io.h>

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s IN_FILE OUT_FILE\n", argv[0]);
    return -1;
  }

  era_tool_configuration_t* tool_trajectory;
  double* timestamps;

  int result = era_read_tool_trajectory(argv[1], &tool_trajectory,
    &timestamps);

  if (result > 0) {
    era_arm_configuration_t arm_trajectory[result];

    era_trajectory_inverse_kinematics(tool_trajectory, result, arm_trajectory);

    result = era_write_arm_trajectory(argv[2], arm_trajectory, timestamps,
      result);
  }

  if (result < 0) era_print_error(stdout, result);

  return 0;
}
