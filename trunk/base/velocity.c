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

#include "velocity.h"

#include "global.h"

const char* era_velocity_errors[] = {
  "success",
  "error opening file",
  "invalid file format",
  "error creating file",
  "error writing file",
};

void era_velocity_init_state(era_velocity_state_p state) {
  memset(state, 0, sizeof(era_velocity_state_t));
}

void era_velocity_init_profile(era_velocity_profile_p profile, ssize_t
  num_points) {
  int i;

  profile->num_points = num_points;
  profile->num_limit_errors = 0;

  if (num_points) {
    profile->points = malloc(num_points*sizeof(era_velocity_profile_t));
    profile->timestamps = malloc(num_points*sizeof(double));

    profile->limit_errors = malloc(num_points*sizeof(int));

    for (i = 0; i < num_points; ++i) {
      era_velocity_init_state(&profile->points[i]);
      profile->timestamps[i] = 0.0;

      profile->limit_errors = ERA_VELOCITY_ERROR_NONE;
    }
  }
  else {
    profile->points = 0;
    profile->timestamps = 0;

    profile->limit_errors = 0;
  }
}

void era_velocity_destroy_profile(era_velocity_profile_p profile) {
  if (profile->num_points) {
    free(profile->points);
    free(profile->timestamps);

    free(profile->limit_errors);

    profile->num_points = 0;
    profile->points = 0;
    profile->timestamps = 0;

    profile->num_limit_errors = 0;
    profile->limit_errors = 0;
  }
}

void era_velocity_print_state(FILE* stream, era_velocity_state_p state) {
  fprintf(stream, "%14s: %8.2f deg/s\n",
    "shoulder_yaw", rad_to_deg(state->shoulder_yaw));
  fprintf(stream, "%14s: %8.2f deg/s\n",
    "shoulder_roll", rad_to_deg(state->shoulder_roll));
  fprintf(stream, "%14s: %8.2f deg/s\n",
    "shoulder_pitch", rad_to_deg(state->shoulder_pitch));
  fprintf(stream, "%14s: %8.2f deg/s\n",
    "elbow_pitch", rad_to_deg(state->elbow_pitch));
  fprintf(stream, "%14s: %8.2f deg/s\n",
    "tool_roll", rad_to_deg(state->tool_roll));
  fprintf(stream, "%14s: %8.2f deg/s\n",
    "tool_opening", rad_to_deg(state->tool_opening));
}

void era_velocity_print_profile(FILE* stream, era_velocity_profile_p profile) {
  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s  %12s\n",
    "shoulder_yaw",
    "shoulder_roll",
    "shoulder_pitch",
    "elbow_pitch",
    "tool_roll",
    "tool_opening",
    "limit_error");

  int i;
  for (i = 0; i < profile->num_points; i++) {
    fprintf(stream,
      "%10.2f deg/s  %10.2f deg/s  %10.2f deg/s  %10.2f deg/s  %10.2f deg/s  "
      "%10.2f deg/s  %12d\n",
      rad_to_deg(profile->points[i].shoulder_yaw),
      rad_to_deg(profile->points[i].shoulder_roll),
      rad_to_deg(profile->points[i].shoulder_pitch),
      rad_to_deg(profile->points[i].elbow_pitch),
      rad_to_deg(profile->points[i].tool_roll),
      rad_to_deg(profile->points[i].tool_opening),
      profile->limit_errors[i]);
  }
}

int era_velocity_read_profile(const char* filename, era_velocity_profile_p
  profile) {
  int i, result;
  FILE* file;
  char buffer[1024];

  era_velocity_state_t point;
  double timestamp;

  era_velocity_init_profile(profile, 0);

  file = fopen(filename, "r");
  if (file == NULL)
    return -ERA_VELOCITY_ERROR_FILE_OPEN;

  while (fgets(buffer, sizeof(buffer), file) != NULL) {
    if (buffer[0] != '#') {
      result = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf",
        &point.shoulder_yaw,
        &point.shoulder_roll,
        &point.shoulder_pitch,
        &point.elbow_pitch,
        &point.tool_roll,
        &point.tool_opening,
        &timestamp);

      if (result < 7) {
        fclose(file);
        return -ERA_VELOCITY_ERROR_FILE_FORMAT;
      }

      profile->points = realloc(profile->points,
        (profile->num_points+1)*sizeof(era_velocity_state_t));
      profile->points[profile->num_points] = point;

      profile->timestamps = realloc(profile->timestamps,
        (profile->num_points+1)*sizeof(double));
      profile->timestamps[profile->num_points] = timestamp;

      profile->limit_errors = realloc(profile->limit_errors,
        (profile->num_points+1)*sizeof(int));
      profile->limit_errors[profile->num_points] = 0;

      ++profile->num_points;
    }
  }

  fclose(file);

  return profile->num_points;
}

int era_velocity_write_profile(const char* filename, era_velocity_profile_p
  profile) {
  int i;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");

  if (file == NULL)
    return -ERA_VELOCITY_ERROR_FILE_CREATE;

  for (i = 0; i < profile->num_points; ++i) {
    sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
      profile->points[i].shoulder_yaw,
      profile->points[i].shoulder_roll,
      profile->points[i].shoulder_pitch,
      profile->points[i].elbow_pitch,
      profile->points[i].tool_roll,
      profile->points[i].tool_opening,
      profile->timestamps[i]);

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_VELOCITY_ERROR_FILE_WRITE;
    }
  }

  fclose(file);

  return profile->num_points;
}
