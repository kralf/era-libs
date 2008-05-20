/*	Kinematic system model for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#include <math.h>

#include "kinematics.h"

#define sqr(x) x*x

const era_arm_geometry_t era_arm_geometry = {
  .upper = 0.2305,
  .lower = 0.224,
  .tool = 0.188,
};

const era_arm_configuration_t era_arm_configuration_min = {
  .shoulder_yaw = -M_PI/2+0.1,
  .shoulder_roll = 0.1,
  .shoulder_pitch = -M_PI/9+0.1,
  .ellbow_pitch = 0.1,
  .tool_roll = -M_PI*8/9+0.1,
  .tool_opening = 0,
};

const era_arm_configuration_t era_arm_configuration_max = {
  .shoulder_yaw = M_PI/6-0.1,
  .shoulder_roll = M_PI/2-0.1,
  .shoulder_pitch = M_PI/2-0.1,
  .ellbow_pitch = M_PI*2/3-0.1,
  .tool_roll = M_PI*8/9-0.1,
  .tool_opening = M_PI/2,
};

void era_print_tool_configuration(
  FILE* stream,
  era_tool_configuration_t* tool_configuration) {
  fprintf(stream, "x:       %f m\n",
    tool_configuration->x);
  fprintf(stream, "y:       %f m\n",
    tool_configuration->y);
  fprintf(stream, "z:       %f m\n",
    tool_configuration->z);
  fprintf(stream, "yaw:     %f°\n",
    tool_configuration->yaw*180/M_PI);
  fprintf(stream, "roll:    %f°\n",
    tool_configuration->roll*180/M_PI);
  fprintf(stream, "opening: %f°\n",
    tool_configuration->opening*180/M_PI);
}

void era_print_arm_configuration(
  FILE* stream,
  era_arm_configuration_t* arm_configuration) {
  fprintf(stream, "shoulder_yaw:   %f°\n",
    arm_configuration->shoulder_yaw*180/M_PI);
  fprintf(stream, "shoulder_roll:  %f°\n",
    arm_configuration->shoulder_roll*180/M_PI);
  fprintf(stream, "shoulder_pitch: %f°\n",
    arm_configuration->shoulder_pitch*180/M_PI);
  fprintf(stream, "ellbow_pitch:   %f°\n",
    arm_configuration->ellbow_pitch*180/M_PI);
  fprintf(stream, "tool_roll:      %f°\n",
    arm_configuration->tool_roll*180/M_PI);
  fprintf(stream, "tool_opening:   %f°\n",
    arm_configuration->tool_opening*180/M_PI);
}

int era_test_arm_configuration_limits(
  era_arm_configuration_t* arm_configuration) {
  int i;
  double* theta = (double*)arm_configuration;
  double* theta_min = (double*)&era_arm_configuration_min;
  double* theta_max = (double*)&era_arm_configuration_max;

  for (i = 0; i <= sizeof(era_arm_configuration_t)/sizeof(double); i++)
    if (theta[i] < theta_min[i] || theta[i] > theta_max[i])
    return 1;

  return 0;
}

void era_forward_kinematics(
  era_arm_configuration_t* arm_configuration,
  era_tool_configuration_t* tool_configuration) {
  double a3 = era_arm_geometry.upper;
  double a4 = era_arm_geometry.lower;
  double a5 = era_arm_geometry.tool;
  double* theta = (double*)arm_configuration;

  tool_configuration->x =
    -a4*cos(theta[3])*sin(theta[0])*sin(theta[2])
    -a4*sin(theta[3])*sin(theta[0])*cos(theta[2])
    +a3*cos(theta[0])*sin(theta[1])*cos(theta[2])
    +a4*cos(theta[3])*cos(theta[0])*sin(theta[1])*cos(theta[2])
    -a3*sin(theta[0])*sin(theta[2])
    -a4*sin(theta[3])*cos(theta[0])*sin(theta[1])*sin(theta[2])
    -sin(theta[0])*a5;

  tool_configuration->y =
    cos(theta[0])*a5+a3*sin(theta[0])*sin(theta[1])*cos(theta[2])
    +a4*sin(theta[3])*cos(theta[0])*cos(theta[2])
    +a4*cos(theta[3])*cos(theta[0])*sin(theta[2])+a3*cos(theta[0])*sin(theta[2])
    +a4*cos(theta[3])*sin(theta[0])*sin(theta[1])*cos(theta[2])
    -a4*sin(theta[3])*sin(theta[0])*sin(theta[1])*sin(theta[2]);

  tool_configuration->z =
    -cos(theta[1])*cos(theta[2])*cos(theta[3])*a4
    +cos(theta[1])*sin(theta[2])*sin(theta[3])*a4
    -cos(theta[1])*cos(theta[2])*a3+a3+a4;

  tool_configuration->yaw = theta[0];

  tool_configuration->roll = theta[4]-theta[1];

  tool_configuration->opening = theta[5];
}

int era_inverse_kinematics(
  era_tool_configuration_t* tool_configuration,
  era_arm_configuration_t* arm_configuration) {
  double x, y, c4;
  double a3 = era_arm_geometry.upper;
  double a4 = era_arm_geometry.lower;
  double a5 = era_arm_geometry.tool;
  double* theta = (double*)arm_configuration;

  era_cartesian_t v_sp;
  era_cartesian_t v_beta1;
  era_cartesian_t v_normal;

  v_beta1.x = -sin(tool_configuration->yaw);
  v_beta1.y = cos(tool_configuration->yaw);
  v_beta1.z = 0;

  v_sp.x = tool_configuration->x-a5*v_beta1.x ;
  v_sp.y = tool_configuration->y-a5*v_beta1.y ;
  v_sp.z = tool_configuration->z-(a3+a4)-a5*v_beta1.z;

  v_normal.x = (v_sp.y*v_beta1.z-v_sp.z*v_beta1.y);
  v_normal.y = (v_sp.z*v_beta1.x-v_sp.x*v_beta1.z);
  v_normal.z = (v_sp.x*v_beta1.y-v_sp.y*v_beta1.x);

  arm_configuration->shoulder_yaw = tool_configuration->yaw;

  arm_configuration->shoulder_roll =
    atan2(fabs(v_normal.z), sqrt(sqr(v_normal.x)+sqr(v_normal.y)));

  x = v_sp.x*sin(theta[1])*cos(theta[0])
      +v_sp.y*sin(theta[1])*sin(theta[0])
      -v_sp.z*cos(theta[1]);
  y = -v_sp.x*sin(theta[0])+v_sp.y*cos(theta[0]);
  c4 = (sqr(x)+sqr(y)-sqr(a3)-sqr(a4))/(2*a4*a3);

  arm_configuration->shoulder_pitch =
    atan2(y, x)-atan2((a4*sqrt(1-sqr(c4))), (a3+a4*c4));

  arm_configuration->ellbow_pitch = atan2(sqrt(1-sqr(c4)), c4);

  arm_configuration->tool_roll = theta[1]+tool_configuration->roll;

  arm_configuration->tool_opening = tool_configuration->opening;

  if (arm_configuration->shoulder_roll > -0.1 &&
    arm_configuration->shoulder_roll < 0.1)
    arm_configuration->shoulder_roll = 0.1;

  return era_test_arm_configuration_limits(arm_configuration) ;
}

void era_set_tool_yaw(
  era_tool_configuration_t* tool_configuration) {
  tool_configuration->yaw = -tan(tool_configuration->x/tool_configuration->y);
}
