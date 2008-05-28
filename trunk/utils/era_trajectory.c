/*      Utility program for ERA trajectory control
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <era.h>

#define ERA_ESCAPE 0x1B

#define ERA_CURSOR_FORWARD 'C'
#define ERA_CURSOR_BACKWARD 'D'
#define ERA_CURSOR_UP 'A'
#define ERA_CURSOR_DOWN 'B'

void print(
  const era_arm_configuration_t* configuration,
  const era_arm_velocity_t* velocity,
  double actual_frequency) {
  era_print_configuration(stdout, configuration, velocity);

  fprintf(stdout, "%s %5.1f Hz\n", "UPDATE FREQUENCY", actual_frequency);
  fprintf(stdout, "%c[%d%c\r", ERA_ESCAPE, 8, ERA_CURSOR_UP);
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s DEV FILE\n", argv[0]);
    return -1;
  }

  era_motors_init(argv[1]);

  era_arm_configuration_t* trajectory;
  double* timestamps;

  int result = era_read_arm_trajectory(argv[2], &trajectory, &timestamps);

  if (result > 0) {
    era_sensors_start(print, 10.0);

    result = era_move_trajectory(trajectory, timestamps, result);

    era_sensors_exit();

    free(trajectory);
    free(timestamps);
  }

  if (result < 0) era_print_error(stdout, result);

  era_motors_close();
  return 0;
}
