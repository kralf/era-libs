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

#include "era.h"

#define ERA_ESCAPE 0x1B

#define ERA_CURSOR_FORWARD 'C'
#define ERA_CURSOR_BACKWARD 'D'
#define ERA_CURSOR_UP 'A'
#define ERA_CURSOR_DOWN 'B'

void print(
  const era_arm_configuration_t* configuration,
  const era_arm_velocity_t* velocity,
  double actual_frequency) {
  era_print_configuration(stdout, configuration, velocity);

  fprintf(stdout, "%s %5.1f Hz\n", "UPDATE FREQUENCY", actual_frequency);
  fprintf(stdout, "%c[%d%c\r", ERA_ESCAPE, 8, ERA_CURSOR_UP);
}

int main(int argc, char **argv) {
  if (argc != 3) {
    fprintf(stderr, "Usage: %s DEV FILE\n", argv[0]);
    return -1;
  }

  era_motors_init(argv[1]);

  era_arm_configuration_t* trajectory;
  double* timestamps;

  int result = era_read_arm_trajectory(argv[2], &trajectory, &timestamps);

  if (result > 0) {
    era_sensors_start(print, 10.0);

    result = era_move_trajectory(trajectory, timestamps, result);

    era_sensors_exit();

    free(trajectory);
    free(timestamps);
  }

  if (result < 0) era_print_error(stdout, result);

  era_motors_close();
  return 0;
}
