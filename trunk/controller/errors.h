/*	Header-file for
 *	BlueBotics ERA-5/1 controller error codes
 *
 * 	Ralf Kaestner    ralf.kaestner@gmail.com
 * 	Last change:     22.5.2008
 */

#ifndef _ERRORS_H
#define _ERRORS_H

#include <stdio.h>

/** \file
  * \brief Error codes
  *
  * Providing error codes and descriptions.
  */

/** \brief Predefined error codes
  */
#define ERA_ERROR_NONE 0
#define ERA_ERROR_LIMITS_EXCEEDED 1

/** \brief Predefined error descriptions
  */
extern const char* era_errors[];
extern const char* era_error_undefined;

/** \brief Return an error description
  * \param[in] error The error code for which a description will be return.
  * \return The error description corresponding to the error code.
  */
const char* era_get_error(
  int error);

/** \brief Print an error description
  * \param[in] stream The output stream that will be used for printing the
  *   error description.
  * \param[in] error The error code for which a description will be printed.
  */
void era_print_error(
  FILE* stream,
  int error);

#endif
