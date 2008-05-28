/*      Utility program for ERA arm and tool configuration output
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <era.h>
#include <timer.h>

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
  if ((argc < 2) || (argc > 3)) {
    fprintf(stderr, "Usage: %s DEV [HZ]\n", argv[0]);
    return -1;
  }

  double frequency = 0.0;
  if (argc == 3) sscanf(argv[2], "%lf", &frequency);

  can_init(argv[1]);

  int result = era_sensors_start(print, frequency);

  if (!result) era_thread_wait_exit(&era_sensors_thread);

  can_close();
  return 0;
}
