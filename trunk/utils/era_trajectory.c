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

  era_tool_configuration_t* trajectory;
  double* timestamps;

  int result = era_read_tool_trajectory(argv[2], &trajectory, &timestamps);

  if (result > 0) {
    printf("%d tool trajectory configurations\n", result);

    double dt = 0.1;
    era_arm_velocity_t* velocity_profile;
    era_trajectory_error_t* errors;

    result = era_trajectory_velocities(trajectory, timestamps, result, dt,
      &velocity_profile, &errors);

    if (result > 0) {
      result = era_write_velocity_profile("vel.txt", &velocity_profile,
        &timestamps, result);

      free(velocity_profile);
      free(errors);
    }

    //era_move_trajectory(tool_configurations, timestamps, 1);

    free(trajectory);
    free(timestamps);
  }

  if (result < 0) era_print_error(stdout, -result);

  era_motors_close();
  return 0;
}
