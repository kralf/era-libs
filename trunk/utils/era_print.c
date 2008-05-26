/*      Utility program for ERA arm and tool configuration output
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <controller.h>

#define ERA_ESCAPE 0x1B

#define ERA_CURSOR_FORWARD 'C'
#define ERA_CURSOR_BACKWARD 'D'
#define ERA_CURSOR_UP 'A'
#define ERA_CURSOR_DOWN 'B'

int main(int argc, char **argv) {
  if (argc != 2) {
    fprintf(stderr, "Usage: %s DEV\n", argv[0]);
    return -1;
  }

  can_init(argv[1]);

  while (1) {
    era_print_configuration(stdout);

    usleep(100000);
    fprintf(stdout, "%c[%d%c\r", ERA_ESCAPE, 7, ERA_CURSOR_UP);
  }

  can_close();
  return 0;
}
