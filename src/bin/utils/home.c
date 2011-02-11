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

#include "era.h"
#include "motors/home.h"

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
  if (!era_security_enable_home(&arm.security, &arm.motors)) {
    if (!era_motors_home_start(&arm.motors, &arm.security)) {
      while (!quit && era_motors_home_wait(&arm.motors, 0.1));
      era_motors_home_stop(&arm.motors);
    }
    era_security_enable(&arm.security, &arm.motors);
  }
  else
    fprintf(stderr, "security error\n");
  era_close(&arm);

  era_destroy(&arm);
  return 0;
}
