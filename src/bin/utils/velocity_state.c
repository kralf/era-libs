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
#include <signal.h>
#include <math.h>

#include "era.h"

int quit = 0;

void era_signaled(int signal) {
  quit = 1;
}

int main(int argc, char **argv) {
  era_arm_t arm;

  if (era_init_arg(&arm, argc, argv, 0, 0))
    return -1;

  signal(SIGINT, era_signaled);

  if (era_open(&arm))
    return -1;
  while (!quit) {
    era_velocity_state_t state;

    era_get_velocity_state(&arm, &state);
    era_velocity_print_state(stdout, &state);
    fprintf(stdout, "%c[6A\r", 0x1B);
  }
  fprintf(stdout, "%c[6B\n", 0x1B);
  era_close(&arm);

  era_destroy(&arm);
  return 0;
}
