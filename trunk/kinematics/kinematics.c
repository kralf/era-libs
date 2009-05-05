/***************************************************************************
 *   Copyright (C) 2008 by Fritz Stoeckli, Ralf Kaestner                   *
 *   stfritz@ethz.ch, ralf.kaestner@gmail.com                              *
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

#include <base/global.h>

#include "kinematics.h"

void era_kinematics_forward_config(era_geometry_p geometry, era_joint_config_p
  joint_config, era_tool_config_p tool_config) {
  double a3 = geometry->upper_length;
  double a4 = geometry->lower_length;
  double a5 = geometry->tool_length;
  double* theta = (double*)joint_config;

  tool_config->x =
    -a4*cos(theta[3])*sin(theta[0])*sin(theta[2])
    -a4*sin(theta[3])*sin(theta[0])*cos(theta[2])
    +a3*cos(theta[0])*sin(theta[1])*cos(theta[2])
    +a4*cos(theta[3])*cos(theta[0])*sin(theta[1])*cos(theta[2])
    -a3*sin(theta[0])*sin(theta[2])
    -a4*sin(theta[3])*cos(theta[0])*sin(theta[1])*sin(theta[2])
    -sin(theta[0])*a5;

  tool_config->y =
    cos(theta[0])*a5+a3*sin(theta[0])*sin(theta[1])*cos(theta[2])
    +a4*sin(theta[3])*cos(theta[0])*cos(theta[2])
    +a4*cos(theta[3])*cos(theta[0])*sin(theta[2])+a3*cos(theta[0])*sin(theta[2])
    +a4*cos(theta[3])*sin(theta[0])*sin(theta[1])*cos(theta[2])
    -a4*sin(theta[3])*sin(theta[0])*sin(theta[1])*sin(theta[2]);

  tool_config->z =
    -cos(theta[1])*cos(theta[2])*cos(theta[3])*a4
    +cos(theta[1])*sin(theta[2])*sin(theta[3])*a4
    -cos(theta[1])*cos(theta[2])*a3+a3+a4;

  tool_config->yaw = theta[0];

  tool_config->roll = theta[4]+theta[1];

  tool_config->opening = theta[5];
}

ssize_t era_kinematics_forward_trajectory(era_geometry_p geometry,
  era_joint_trajectory_p joint_trajectory, era_tool_trajectory_p
  tool_trajectory) {
  int i;
  ssize_t num_points = min(joint_trajectory->num_points,
    tool_trajectory->num_points);

  for (i = 0; i < num_points; i++)
    era_kinematics_forward_config(geometry, &joint_trajectory->points[i],
      &tool_trajectory->points[i]);

  return num_points;
}

void era_kinematics_inverse_config(era_geometry_p geometry, era_tool_config_p
  tool_config, era_joint_config_p joint_config) {
  double x, y, c4;
  double a3 = geometry->upper_length;
  double a4 = geometry->lower_length;
  double a5 = geometry->tool_length;
  double* theta = (double*)joint_config;

  double v_sp_x, v_sp_y, v_sp_z;
  double v_beta1_x, v_beta1_y, v_beta1_z;
  double v_normal_x, v_normal_y, v_normal_z;

  v_beta1_x = -sin(tool_config->yaw);
  v_beta1_y = cos(tool_config->yaw);
  v_beta1_z = 0;

  v_sp_x = tool_config->x-a5*v_beta1_x ;
  v_sp_y = tool_config->y-a5*v_beta1_y ;
  v_sp_z = tool_config->z-(a3+a4)-a5*v_beta1_z;

  v_normal_x = (v_sp_y*v_beta1_z-v_sp_z*v_beta1_y);
  v_normal_y = (v_sp_z*v_beta1_x-v_sp_x*v_beta1_z);
  v_normal_z = (v_sp_x*v_beta1_y-v_sp_y*v_beta1_x);

  joint_config->shoulder_yaw = tool_config->yaw;

  joint_config->shoulder_roll =
    atan2(v_normal_z, sqrt(sqr(v_normal_x)+sqr(v_normal_y)));

  x = v_sp_x*sin(theta[1])*cos(theta[0])
      +v_sp_y*sin(theta[1])*sin(theta[0])
      -v_sp_z*cos(theta[1]);
  y = -v_sp_x*sin(theta[0])+v_sp_y*cos(theta[0]);
  c4 = (sqr(x)+sqr(y)-sqr(a3)-sqr(a4))/(2*a4*a3);

  joint_config->shoulder_pitch =
    atan2(y, x)-atan2((a4*sqrt(1-sqr(c4))), (a3+a4*c4));

  joint_config->ellbow_pitch = atan2(sqrt(1-sqr(c4)), c4);

  joint_config->tool_roll = tool_config->roll-theta[1];

  joint_config->tool_opening = tool_config->opening;

  if (joint_config->shoulder_roll > -0.1 &&
    joint_config->shoulder_roll < 0.1)
    joint_config->shoulder_roll = 0.1;
}

ssize_t era_kinematics_inverse_trajectory(era_geometry_p geometry,
  era_tool_trajectory_p tool_trajectory, era_joint_trajectory_p
  joint_trajectory) {
  int i;
  ssize_t num_points = min(tool_trajectory->num_points,
    joint_trajectory->num_points);

  for (i = 0; i < num_points; i++)
    era_kinematics_inverse_config(geometry, &tool_trajectory->points[i],
      &joint_trajectory->points[i]);

  return num_points;
}

void era_kinematics_set_config_yaw(era_tool_config_p tool_config) {
  tool_config->yaw = -tan(tool_config->x/tool_config->y);
}
