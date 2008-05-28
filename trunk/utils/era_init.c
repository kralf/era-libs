/*      Utility program for ERA initialization
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     19.5.2008
 */

#include <era.h>

int main(int argc, char **argv) {
  if (argc < 2) {
    fprintf(stderr, "Usage: %s DEV\n", argv[0]);
    return -1;
  }

  int result = era_init(argv[1]);

  if (result) era_print_error(stdout, result);

  era_close();
  return 0;
}
