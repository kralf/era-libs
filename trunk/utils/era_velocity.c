/*      Utility program for generating ERA velocity profiles from
 *      arm trajectories
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     28.5.2008
 */

#include <math.h>

#include <trajectory.h>
#include <io.h>

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s IN_FILE OUT_FILE\n", argv[0]);
    return -1;
  }

  era_arm_configuration_t* trajectory;
  double* timestamps;

  int result = era_read_arm_trajectory(argv[1], &trajectory, &timestamps);

  if (result > 0) {
    era_arm_velocity_t velocities[result];

    era_velocity_profile(trajectory, timestamps, result, velocities);

    result = era_write_velocity_profile(argv[2], velocities, timestamps,
      result);

    free(trajectory);
    free(timestamps);
  }

  if (result < 0) era_print_error(stdout, result);

  return 0;
}
