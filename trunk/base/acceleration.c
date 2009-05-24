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

#include <string.h>

#include "acceleration.h"

#include "global.h"

const char* era_acceleration_errors[] = {
  "success",
};

void era_acceleration_init_state(era_acceleration_state_p state) {
  memset(state, 0, sizeof(era_acceleration_state_t));
}

void era_acceleration_print_state(FILE* stream, era_acceleration_state_p 
  state) {
  fprintf(stream, "%14s: % 8.2f °/s^2\n",
    "shoulder_yaw", rad_to_deg(state->shoulder_yaw));
  fprintf(stream, "%14s: % 8.2f °/s^2\n",
    "shoulder_roll", rad_to_deg(state->shoulder_roll));
  fprintf(stream, "%14s: % 8.2f °/s^2\n",
    "shoulder_pitch", rad_to_deg(state->shoulder_pitch));
  fprintf(stream, "%14s: % 8.2f °/s^2\n",
    "elbow_pitch", rad_to_deg(state->elbow_pitch));
  fprintf(stream, "%14s: % 8.2f °/s^2\n",
    "tool_roll", rad_to_deg(state->tool_roll));
  fprintf(stream, "%14s: % 8.2f °/s^2\n",
    "tool_opening", rad_to_deg(state->tool_opening));
}
