/***************************************************************************
 *   Copyright (C) 2004 by Ralf Kaestner                                   *
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

#include <stdio.h>

#include <base/era.h>

int main(int argc, char **argv) {
  era_joint_state_t joint_state;
  if (argc < 8) {
    fprintf(stderr, "usage: %s SHOULDER_YAW SHOULDER_ROLL SHOULDER_PITCH "
      " ELBOW_PITCH TOOL_ROLL TOOL_OPENING VEL [PARAMS]\n", argv[0]);
    return -1;
  }
  joint_state.shoulder_yaw = deg_to_rad(atof(argv[1]));
  joint_state.shoulder_roll = deg_to_rad(atof(argv[2]));
  joint_state.shoulder_pitch = deg_to_rad(atof(argv[3]));
  joint_state.elbow_pitch = deg_to_rad(atof(argv[4]));
  joint_state.tool_roll = deg_to_rad(atof(argv[5]));
  joint_state.tool_opening = deg_to_rad(atof(argv[6]));
  float vel_factor = atof(argv[7]);

  era_arm_t arm;
  era_init_arg(&arm, argc, argv, 0);

  if (era_open(&arm))
    return -1;
  int result;
  if (result = era_move_joints(&arm, &joint_state, vel_factor))
    printf("%s\n", era_errors[result]);
  era_close(&arm);

  era_destroy(&arm);
  return 0;
}
