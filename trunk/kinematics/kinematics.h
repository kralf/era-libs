/*	Header-file for
 *      Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#ifndef _KINEMATICS_H
#define _KINEMATICS_H

#include <stdio.h>

/** \file
  * \brief Kinematic system model
  * Inverse and forward calculations for BlueBotics ERA-5/1 robot arm.
  */

/** \brief Structure defining a cartesian vector */
typedef struct {
  double x;  //!< The cartesian X-coordinate [m].
  double y;  //!< The cartesian Y-coordinate [m].
  double z;  //!< The cartesian Z-coordinate [m].
} era_cartesian_t;

/** \brief Structure defining the arm geometry */
typedef struct {
  double upper;  //!< The upper arm's length [m].
  double lower;  //!< The forearm's length [m].
  double tool;   //!< The tool's length [m].
} era_arm_geometry_t;

/** \brief Structure defining the tool configuration */
typedef struct {
  double x;        //!< The tool's X-coordinate [m].
  double y;        //!< The tool's Y-coordinate [m].
  double z;        //!< The tool's Z-coordinate [m].
  double yaw;      //!< The tool's yaw angle [rad], denoted beta1.
  double roll;     //!< The tool's roll angle [rad], denoted beta2.
  double opening;  //!< The tool's opening angle [rad].
} era_tool_configuration_t;

/** \brief Structure defining the arm configuration */
typedef struct {
  double shoulder_yaw;    //!< The shoulder's yaw angle [rad], denoted theta1.
  double shoulder_roll;   //!< The shoulder's roll angle [rad], denoted theta2.
  double shoulder_pitch;  //!< The shoulder's pitch angle [rad], denoted theta3.
  double ellbow_pitch;    //!< The ellbow's pitch angle [rad], denoted theta4.
  double tool_roll;       //!< The tool's roll angle [rad], denoted theta6.
  double tool_opening;    //!< The tool's opening angle [rad].
} era_arm_configuration_t;

/** \brief Constant defining the arm's geometry */
extern const era_arm_geometry_t era_arm_geometry;

/** \brief Structure holding the lower limit of the arm configuration space */
extern era_arm_configuration_t era_arm_configuration_min;
/** \brief Structure holding the upper limit of the arm configuration space */
extern era_arm_configuration_t era_arm_configuration_max;
/** \brief Variable holding the safety margin of the arm configuration space */
extern double era_arm_configuration_safety_margin;

/** \brief Initialize a tool configuration
  * \param[in] tool_configuration The tool configuration to be initialized
  *   with 0.
  */
void era_init_tool_configuration(
  era_tool_configuration_t* tool_configuration);

/** \brief Initialize an arm configuration
  * \param[in] arm_configuration The arm configuration to be initialized
  *   with 0.
  */
void era_init_arm_configuration(
  era_arm_configuration_t* arm_configuration);

/** \brief Print a tool configuration
  * \param[in] stream The output stream that will be used for printing the
  *   tool configuration.
  * \param[in] tool_configuration The tool configuration that will be printed.
  */
void era_print_tool_configuration(
  FILE* stream,
  const era_tool_configuration_t* tool_configuration);

/** \brief Print an arm configuration
  * \param[in] stream The output stream that will be used for printing the
  *   arm configuration.
  * \param[in] arm_configuration The arm configuration that will be printed.
  */
void era_print_arm_configuration(
  FILE* stream,
  const era_arm_configuration_t* arm_configuration);

/** \brief Initialize the kinematic model
  * \param[in] arm_configuration_min The lower limit of the arm configuration
  *    space.
  * \param[in] arm_configuration_max The upper limit of the arm configuration
  *    space.
  */
void era_kinematics_init(
  const era_arm_configuration_t* arm_configuration_min,
  const era_arm_configuration_t* arm_configuration_max);

/** \brief Test an arm configuration against configuration space limits
  * \param[in] arm_configuration The arm configuration that will be tested
  *   against the configuration space limits. If null, the result will
  *   always be ERA_ERROR_NONE.
  * \return The resulting error code.
  */
int era_test_arm_configuration_limits(
  const era_arm_configuration_t* arm_configuration);

/** \brief Forward kinematic calculations
  * \param[in] arm_configuration The arm configuration for which the tool
  *   configuration will be calculated.
  * \param[out] tool_configuration The tool configuration that results from
  *   the forward kinematic calculations.
  */
void era_forward_kinematics(
  const era_arm_configuration_t* arm_configuration,
  era_tool_configuration_t* tool_configuration);

/** \brief Inverse kinematic calculations
  * \param[in] tool_configuration The tool configuration for which the arm
  *   configuration will be calculated.
  * \param[out] arm_configuration The arm configuration that results from
  *   the inverse kinematic calculations.
  */
void era_inverse_kinematics(
  const era_tool_configuration_t* tool_configuration,
  era_arm_configuration_t* arm_configuration);

/** \brief Calculate and set the tool's yaw angle
  * \param[in,out] tool_configuration The tool configuration for which the
  *   yaw angle will be calculated and set.
  */
void era_set_tool_yaw(
  era_tool_configuration_t* tool_configuration);

#endif
