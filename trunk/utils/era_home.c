/*      Utility program for ERA homing
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <controller.h>

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s DEV\n", argv[0]);
    return -1;
  }

  era_init(argv[1], ERA_HOMING_MODE_NONE);

  era_move_home();

  era_close();
  return 0;
}
