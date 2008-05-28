/*	Header-file for
 *      Trajectory generation for BlueBotics ERA-5/1
 *
 *      Ralf Kaestner    ralf.kaestner@gmail.com
 *      Last change:     28.5.2008
 */

#ifndef _TRAJECTORY_H
#define _TRAJECTORY_H

#include "kinematics.h"
/** \file
  * \brief Kinematic trajectory generation
  * Generates trajectories in arm space for given tool trajectories.
  */

/** \brief Print a tool trajectory
  * \param[in] stream The output stream that will be used for printing the
  *   tool trajectory.
  * \param[in] tool_trajectory The tool trajectory that will be printed.
  * \param[in] num_configurations The number of tool configurations
  *   in the trajectory.
  */
void era_print_tool_trajectory(
  FILE* stream,
  const era_tool_configuration_t* tool_trajectory,
  int num_configurations);

/** \brief Print an arm trajectory
  * \param[in] stream The output stream that will be used for printing the
  *   arm trajectory.
  * \param[in] arm_trajectory The arm trajectory that will be printed.
  * \param[in] num_configurations The number of arm configurations
  *   in the trajectory.
  */
void era_print_arm_trajectory(
  FILE* stream,
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations);

/** \brief Test an arm trajectory against configuration space limits
  * \param[in] arm_trajectory The arm trajectory that will be tested
  *   against the configuration space limits. If null, the result will
  *   always be ERA_ERROR_NONE.
  * \param[in] num_configurations The number of arm configurations
  *   in the trajectory.
  * \return The resulting error code.
  */
int era_test_trajectory_limits(
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations);

/** \brief Trajectory forward kinematic calculations
  * \param[in] arm_trajectory The arm trajectory for which the tool
  *   trajectory will be calculated.
  * \param[in] num_configurations The number of arm configurations
  *   in the trajectory.
  * \param[out] tool_trajectory The tool trajectory that results from
  *   the forward kinematic calculations.
  */
void era_trajectory_forward_kinematics(
  const era_arm_configuration_t* arm_trajectory,
  int num_configurations,
  era_tool_configuration_t* tool_trajectory);

/** \brief Trajectory inverse kinematic calculations
  * \param[in] tool_trajectory The tool trajectory for which the arm
  *   trajectory will be calculated.
  * \param[in] num_configurations The number of tool configurations
  *   in the trajectory.
  * \param[out] arm_trajectory The arm trajectory that results from
  *   the inverse kinematic calculations.
  */
void era_trajectory_inverse_kinematics(
  const era_tool_configuration_t* tool_trajectory,
  int num_configurations,
  era_arm_configuration_t* arm_trajectory);

#endif
