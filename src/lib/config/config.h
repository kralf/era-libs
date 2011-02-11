/***************************************************************************
 *   Copyright (C) 2008 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef ERA_CONFIG_H
#define ERA_CONFIG_H

/** \brief ERA configuration methods
  * Configuration methods for the BlueBotics ERA-5/1.
  */

#include <tulibs/config.h>

#include "config/parameters.h"

/** \brief Predefined ERA configuration constants
  */
#define ERA_CONFIG_ARG_PREFIX                   "era"

#define ERA_CONFIG_ARG_PREFIX_ARM               "arm"
#define ERA_CONFIG_ARG_PREFIX_SHOULDER_YAW      "shoulder-yaw"
#define ERA_CONFIG_ARG_PREFIX_SHOULDER_ROLL     "shoulder-roll"
#define ERA_CONFIG_ARG_PREFIX_SHOULDER_PITCH    "shoulder-pitch"
#define ERA_CONFIG_ARG_PREFIX_ELBOW_PITCH       "elbow-pitch"
#define ERA_CONFIG_ARG_PREFIX_TOOL_ROLL         "tool-roll"
#define ERA_CONFIG_ARG_PREFIX_TOOL_OPENING      "tool-opening"

/** \brief Predefined joint configuration prefixes
  */
extern const char* era_config_joint_prefixes[];

/** \brief Structure defining the arm's joint configuration
  */
typedef struct era_config_joint_t {
  config_t shoulder_yaw;        //!< The shoulder yaw joint configuration.
  config_t shoulder_roll;       //!< The shoulder roll joint configuration.
  config_t shoulder_pitch;      //!< The shoulder pitch joint configuration.

  config_t elbow_pitch;         //!< The elbow pitch joint configuration.

  config_t tool_roll;           //!< The tool roll joint configuration.
  config_t tool_opening;        //!< The tool opening joint configuration.
} era_config_joint_t, *era_config_joint_p;

/** \brief Structure defining the arm's configuration
  */
typedef struct era_config_t {
  config_t arm;                 //!< The arm's configuration parameters.
  era_config_joint_t joints;    //!< The arm's joint configuration parameters.
} era_config_t, *era_config_p;

/** \brief Predefined ERA default configuration
  */
extern era_config_t era_config_default;

/** \brief Initialize an empty arm configuration
  * \param[in] config The arm configuration to be initialized.
  */
void era_config_init(
  era_config_p config);

/** \brief Initialize an arm configuration from default parameters
  * \param[in] config The arm configuration to be initialized.
  * \param[in] default_config The default arm configuration parameters used to
  *   initialize the arm configuration.
  */
void era_config_init_default(
  era_config_p config,
  era_config_p default_config);

/** \brief Initialize an arm's configuration from command line arguments
  * \param[in] config The arm configuration to be initialized.
  * \param[in] argc The number of supplied command line arguments.
  * \param[in] argv The list of supplied command line arguments.
  * \param[in] prefix An optional argument prefix that will be stripped from 
  *   the parameter keys.
  * \param[in] args An optional string naming the expected arguments.
  * \return The resulting configuration error code.
  */
int era_config_init_arg(
  era_config_p config,
  int argc,
  char **argv,
  const char* prefix,
  const char* args);

/** \brief Destroy an arm configuration
  * \param[in] config The arm configuration to be destroyed.
  */
void era_config_destroy(
  era_config_p config);

/** \brief Print an arm configuration
  * \param[in] stream The output stream that will be used for printing the
  *   arm configuration.
  * \param[in] config The arm configuration that will be printed.
  */
void era_config_print(
  FILE* stream,
  era_config_p config);

/** \brief Print help for an arm configuration
  * \param[in] stream The output stream that will be used for printing the
  *   arm configuration help.
  * \param[in] config The arm configuration for which help will be printed.
  * \param[in] prefix An optional argument prefix that will be prepended to
  *   the parameter keys.
  */
void era_config_print_help(
  FILE* stream,
  era_config_p config,
  const char* prefix);

/** \brief Set arm configuration parameters from a source arm configuration.
  * \param[in] dst_config The arm configuration to set the parameters for.
  * \param[in] src_config The arm configuration containing the source 
  *   parameters to be set.
  */
void era_config_set(
  era_config_p dst_config,
  era_config_p src_config);

/** \brief Retrieve an arm's configuration parameter's string values
  * \param[in] config The arm configuration to retrieve the string 
  *   values from.
  * \param[in] key The key of the string values to be retrieved.
  * \param[out] values The retrieved string values.
  * \return The retrieved string values as passed to the function.
  */
const char** era_config_joint_get_string(
  era_config_p config,
  const char* key,
  const char** values);

/** \brief Retrieve an arm's configuration parameter's integer values
  * \param[in] config The arm configuration to retrieve the integer 
  *   values from.
  * \param[in] key The key of the integer values to be retrieved.
  * \param[out] values The retrieved integer values.
  * \return The retrieved integer values as passed to the function.
  */
int* era_config_joint_get_int(
  era_config_p config,
  const char* key,
  int* values);

/** \brief Retrieve an arm's configuration parameter's floating point values
  * \param[in] config The arm configuration to retrieve the floating point 
  *   values from.
  * \param[in] key The key of the floating point values to be retrieved.
  * \param[out] values The retrieved floating point values.
  * \return The retrieved floating point values as passed to the function.
  */
double* era_config_joint_get_float(
  era_config_p config,
  const char* key,
  double* values);

/** \brief Retrieve an arm's configuration parameter's radian space values
  * \param[in] config The arm configuration to retrieve the radian space
  *   values from.
  * \param[in] key The key of the radian space values to be retrieved.
  * \param[out] values The retrieved radian space values.
  * \return The retrieved radian space values as passed to the function.
  */
double* era_config_joint_get_rad(
  era_config_p config,
  const char* key,
  double* values);

#endif
