/*      Utility program for ERA simulated trajectory control
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <era.h>

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s FILE\n", argv[0]);
    return -1;
  }

  era_arm_configuration_t* trajectory;
  double* timestamps;

  int result = era_read_arm_trajectory(argv[1], &trajectory, &timestamps);

  if (result > 0) {
    //result = era_move_trajectory(trajectory, timestamps, result);

    free(trajectory);
    free(timestamps);
  }

  if (result < 0) era_print_error(stdout, result);

  return 0;
}
