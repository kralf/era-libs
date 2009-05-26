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

void era_kinematics_forward_state(era_geometry_p geometry, era_joint_state_p
  joint_state, era_tool_state_p tool_state) {
  double a3 = geometry->upper_length;
  double a4 = geometry->lower_length;
  double a5 = geometry->tool_length;
  double* theta = (double*)joint_state;

  tool_state->x =
    -a4*cos(theta[3])*sin(theta[0])*sin(theta[2])
    -a4*sin(theta[3])*sin(theta[0])*cos(theta[2])
    +a3*cos(theta[0])*sin(theta[1])*cos(theta[2])
    +a4*cos(theta[3])*cos(theta[0])*sin(theta[1])*cos(theta[2])
    -a3*sin(theta[0])*sin(theta[2])
    -a4*sin(theta[3])*cos(theta[0])*sin(theta[1])*sin(theta[2])
    -sin(theta[0])*a5;

  tool_state->y =
    cos(theta[0])*a5+a3*sin(theta[0])*sin(theta[1])*cos(theta[2])
    +a4*sin(theta[3])*cos(theta[0])*cos(theta[2])
    +a4*cos(theta[3])*cos(theta[0])*sin(theta[2])+a3*cos(theta[0])*sin(theta[2])
    +a4*cos(theta[3])*sin(theta[0])*sin(theta[1])*cos(theta[2])
    -a4*sin(theta[3])*sin(theta[0])*sin(theta[1])*sin(theta[2]);

  tool_state->z =
    -cos(theta[1])*cos(theta[2])*cos(theta[3])*a4
    +cos(theta[1])*sin(theta[2])*sin(theta[3])*a4
    -cos(theta[1])*cos(theta[2])*a3+a3+a4;

  tool_state->yaw = theta[0];

  tool_state->roll = theta[4]+theta[1];

  tool_state->opening = theta[5];
}

ssize_t era_kinematics_forward_path(era_geometry_p geometry, era_joint_path_p 
  joint_path, era_tool_path_p tool_path) {
  int i;
  ssize_t num_points = min(joint_path->num_points, tool_path->num_points);

  for (i = 0; i < num_points; i++)
    era_kinematics_forward_state(geometry, &joint_path->points[i],
      &tool_path->points[i]);

  return num_points;
}

void era_kinematics_inverse_state(era_geometry_p geometry, era_tool_state_p
  tool_state, era_joint_state_p joint_state) {
  double x, y, c4;
  double a3 = geometry->upper_length;
  double a4 = geometry->lower_length;
  double a5 = geometry->tool_length;
  double* theta = (double*)joint_state;

  double v_sp_x, v_sp_y, v_sp_z;
  double v_beta1_x, v_beta1_y, v_beta1_z;
  double v_normal_x, v_normal_y, v_normal_z;

  v_beta1_x = -sin(tool_state->yaw);
  v_beta1_y = cos(tool_state->yaw);
  v_beta1_z = 0;

  v_sp_x = tool_state->x-a5*v_beta1_x ;
  v_sp_y = tool_state->y-a5*v_beta1_y ;
  v_sp_z = tool_state->z-(a3+a4)-a5*v_beta1_z;

  v_normal_x = (v_sp_y*v_beta1_z-v_sp_z*v_beta1_y);
  v_normal_y = (v_sp_z*v_beta1_x-v_sp_x*v_beta1_z);
  v_normal_z = (v_sp_x*v_beta1_y-v_sp_y*v_beta1_x);

  joint_state->shoulder_yaw = tool_state->yaw;

  joint_state->shoulder_roll =
    atan2(v_normal_z, sqrt(sqr(v_normal_x)+sqr(v_normal_y)));

  x = v_sp_x*sin(theta[1])*cos(theta[0])
      +v_sp_y*sin(theta[1])*sin(theta[0])
      -v_sp_z*cos(theta[1]);
  y = -v_sp_x*sin(theta[0])+v_sp_y*cos(theta[0]);
  c4 = (sqr(x)+sqr(y)-sqr(a3)-sqr(a4))/(2*a4*a3);

  joint_state->shoulder_pitch =
    atan2(y, x)-atan2((a4*sqrt(1-sqr(c4))), (a3+a4*c4));

  joint_state->elbow_pitch = atan2(sqrt(1-sqr(c4)), c4);

  joint_state->tool_roll = tool_state->roll-theta[1];

  joint_state->tool_opening = tool_state->opening;

  if (joint_state->shoulder_roll > -0.1 &&
    joint_state->shoulder_roll < 0.1)
    joint_state->shoulder_roll = 0.1;
}

ssize_t era_kinematics_inverse_path(era_geometry_p geometry, era_tool_path_p 
  tool_path, era_joint_path_p joint_path) {
  int i;
  ssize_t num_points = min(tool_path->num_points, joint_path->num_points);

  for (i = 0; i < num_points; i++)
    era_kinematics_inverse_state(geometry, &tool_path->points[i],
      &joint_path->points[i]);

  return num_points;
}

void era_kinematics_set_state_yaw(era_tool_state_p tool_state) {
  tool_state->yaw = -tan(tool_state->x/tool_state->y);
}
