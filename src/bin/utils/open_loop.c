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
#include "control/open_loop.h"

thread_t control_thread;

int quit = 0;

void era_signaled(int signal) {
  era_control_open_loop_exit(&control_thread);
}

int main(int argc, char **argv) {
  int result;
  thread_mutex_t mutex;
  era_arm_t arm;
  era_trajectory_t trajectory;

  if (era_init_arg(&arm, argc, argv, 0, "FILE FREQUENCY"))
    return -1;
  const char* file = argv[1];
  float freq = atof(argv[2]);

  thread_mutex_init(&mutex);
  if ((result = era_trajectory_read(file, &trajectory)) < 0) {
    fprintf(stderr, "%s\n", era_trajectory_errors[-result]);
    return -1;
  }

  signal(SIGINT, era_signaled);

  if (era_open(&arm))
    return -1;
  if (!(result = era_control_open_loop_start(&control_thread, &arm, 
    &mutex, &trajectory, freq)))
    thread_wait_exit(&control_thread);
  else
    fprintf(stderr, "%s\n", era_control_open_loop_errors[result]);
  era_close(&arm);

  era_destroy(&arm);
  thread_mutex_destroy(&mutex);
  return 0;
}
