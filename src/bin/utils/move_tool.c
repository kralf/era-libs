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

#include "era.h"

int main(int argc, char **argv) {
  era_arm_t arm;
  era_tool_state_t tool_state;

  if (era_init_arg(&arm, argc, argv, 0, "X Y Z YAW ROLL OPENING VELOCITY"))
    return -1;
  tool_state.x = atof(argv[1]);
  tool_state.y = atof(argv[2]);
  tool_state.z = atof(argv[3]);
  tool_state.yaw = deg_to_rad(atof(argv[4]));
  tool_state.roll = deg_to_rad(atof(argv[5]));
  tool_state.opening = deg_to_rad(atof(argv[6]));
  float vel_factor = atof(argv[7]);

  if (era_open(&arm))
    return -1;
  int result;
  if (result = era_move_tool(&arm, &tool_state, vel_factor))
    fprintf(stderr, "%s\n", era_errors[result]);
  era_close(&arm);

  era_destroy(&arm);
  return 0;
}
