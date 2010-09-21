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

#include "base/joint.h"

#include "base/global.h"

const char* era_joint_errors[] = {
  "success",
  "error opening file",
  "invalid file format",
  "error creating file",
  "error writing file",
};

void era_joint_init_state(era_joint_state_p state) {
  memset(state, 0, sizeof(era_joint_state_t));
}

void era_joint_init_path(era_joint_path_p path,
  ssize_t num_points) {
  int i;

  path->num_points = num_points;
  path->num_limit_errors = 0;

  if (num_points) {
    path->points = malloc(num_points*sizeof(era_joint_path_t));
    path->timestamps = malloc(num_points*sizeof(double));

    path->limit_errors = malloc(num_points*sizeof(int));

    for (i = 0; i < num_points; ++i) {
      era_joint_init_state(&path->points[i]);
      path->timestamps[i] = 0.0;

      path->limit_errors = ERA_JOINT_ERROR_NONE;
    }
  }
  else {
    path->points = 0;
    path->timestamps = 0;

    path->limit_errors = 0;
  }
}

void era_joint_destroy_path(era_joint_path_p path) {
  if (path->num_points) {
    free(path->points);
    free(path->timestamps);

    free(path->limit_errors);

    path->num_points = 0;
    path->points = 0;
    path->timestamps = 0;

    path->num_limit_errors = 0;
    path->limit_errors = 0;
  }
}

void era_joint_print_state(FILE* stream, era_joint_state_p state) {
  fprintf(stream, "%14s: %8.2f deg\n",
    "shoulder_yaw", rad_to_deg(state->shoulder_yaw));
  fprintf(stream, "%14s: %8.2f deg\n",
    "shoulder_roll", rad_to_deg(state->shoulder_roll));
  fprintf(stream, "%14s: %8.2f deg\n",
    "shoulder_pitch", rad_to_deg(state->shoulder_pitch));
  fprintf(stream, "%14s: %8.2f deg\n",
    "elbow_pitch", rad_to_deg(state->elbow_pitch));
  fprintf(stream, "%14s: %8.2f deg\n",
    "tool_roll", rad_to_deg(state->tool_roll));
  fprintf(stream, "%14s: %8.2f deg\n",
    "tool_opening", rad_to_deg(state->tool_opening));
}

void era_joint_print_path(FILE* stream, era_joint_path_p path) {
  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s  %12s\n",
    "shoulder_yaw",
    "shoulder_roll",
    "shoulder_pitch",
    "elbow_pitch",
    "tool_roll",
    "tool_opening",
    "limit_error");

  int i;
  for (i = 0; i < path->num_points; i++) {
    fprintf(stream,
      "%12.2f deg  %12.2f deg  %12.2f deg  %12.2f deg  %12.2f deg  "
      "%12.2f deg  %12d\n",
      rad_to_deg(path->points[i].shoulder_yaw),
      rad_to_deg(path->points[i].shoulder_roll),
      rad_to_deg(path->points[i].shoulder_pitch),
      rad_to_deg(path->points[i].elbow_pitch),
      rad_to_deg(path->points[i].tool_roll),
      rad_to_deg(path->points[i].tool_opening),
      path->limit_errors[i]);
  }
}

int era_joint_read_path(const char* filename, era_joint_path_p path) {
  int i, result;
  FILE* file;
  char buffer[1024];

  era_joint_state_t point;
  double timestamp;

  era_joint_init_path(path, 0);

  file = fopen(filename, "r");
  if (file == NULL)
    return -ERA_JOINT_ERROR_FILE_OPEN;

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

      if (result < 6) {
        fclose(file);
        return -ERA_JOINT_ERROR_FILE_FORMAT;
      }

      path->points = realloc(path->points,
        (path->num_points+1)*sizeof(era_joint_state_t));
      path->points[path->num_points] = point;

      if (result == 7) {
        path->timestamps = realloc(path->timestamps,
          (path->num_points+1)*sizeof(double));
        path->timestamps[path->num_points] = timestamp;
      }

      path->limit_errors = realloc(path->limit_errors,
        (path->num_points+1)*sizeof(int));
      path->limit_errors[path->num_points] = 0;

      ++path->num_points;
    }
  }

  fclose(file);

  return path->num_points;
}

int era_joint_write_path(const char* filename, era_joint_path_p path) {
  int i;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");
  if (file == NULL)
    return -ERA_JOINT_ERROR_FILE_CREATE;

  for (i = 0; i < path->num_points; ++i) {
    sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
      path->points[i].shoulder_yaw,
      path->points[i].shoulder_roll,
      path->points[i].shoulder_pitch,
      path->points[i].elbow_pitch,
      path->points[i].tool_roll,
      path->points[i].tool_opening,
      path->timestamps[i]);

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_JOINT_ERROR_FILE_WRITE;
    }
  }

  fclose(file);

  return path->num_points;
}
