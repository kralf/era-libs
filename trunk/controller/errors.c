/*      BlueBotics ERA-5/1 controller error codes
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     22.5.2008
 */

#include <stdlib.h>

#include "errors.h"

const char* era_errors[] = {
  "no error",
  "invalid configuration",
  "configuration space limits exceeded",
  "unsupported operation mode",
};

const char* era_error_undefined = "undefined";

const char* era_get_error(
  int error) {
  if (error < sizeof(era_errors)/sizeof(const char*))
    return era_errors[error];
  else
    return era_error_undefined;
}

void era_print_error(
  FILE* stream,
  int error) {
  fprintf(stream, "Error: %s\n", era_get_error(error));
}
