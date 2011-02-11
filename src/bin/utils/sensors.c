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
#include "control/sensors.h"

int quit = 0;

void era_signaled(int signal) {
  quit = 1;
}

void era_sensors_handle(era_joint_state_p joint_state, era_velocity_state_p 
  vel_state, double frequency) {
  era_joint_print_state(stdout, joint_state);
  fprintf(stdout, "sensor thread frequency %5.1f Hz\n", frequency);
  fprintf(stdout, "%c[7A\r", 0x1B);
}

int main(int argc, char **argv) {
  thread_t thread;
  thread_mutex_t mutex;
  era_arm_t arm;

  if (era_init_arg(&arm, argc, argv, 0, "FREQUENCY"))
    return -1;
  float freq = atof(argv[1]);

  thread_mutex_init(&mutex);
  signal(SIGINT, era_signaled);

  if (era_open(&arm))
    return -1;
  era_control_sensors_start(&thread, &arm, &mutex, era_sensors_handle, freq);
  while (!quit) timer_sleep(0.1);
  era_control_sensors_exit(&thread);
  fprintf(stdout, "%c[7B\n", 0x1B);
  era_close(&arm);

  era_destroy(&arm);
  thread_mutex_destroy(&mutex);
  return 0;
}
