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

#include "joint.h"

#include "global.h"

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

void era_joint_init_trajectory(era_joint_trajectory_p trajectory,
  ssize_t num_points) {
  int i;

  trajectory->num_points = num_points;
  trajectory->num_limit_errors = 0;

  if (num_points) {
    trajectory->points = malloc(num_points*sizeof(era_joint_trajectory_t));
    trajectory->timestamps = malloc(num_points*sizeof(double));

    trajectory->limit_errors = malloc(num_points*sizeof(int));

    for (i = 0; i < num_points; ++i) {
      era_joint_init_state(&trajectory->points[i]);
      trajectory->timestamps[i] = 0.0;

      trajectory->limit_errors = ERA_JOINT_ERROR_NONE;
    }
  }
  else {
    trajectory->points = 0;
    trajectory->timestamps = 0;

    trajectory->limit_errors = 0;
  }
}

void era_joint_destroy_trajectory(era_joint_trajectory_p trajectory) {
  if (trajectory->num_points) {
    free(trajectory->points);
    free(trajectory->timestamps);

    free(trajectory->limit_errors);

    trajectory->num_points = 0;
    trajectory->points = 0;

    trajectory->num_limit_errors = 0;
    trajectory->limit_errors = 0;
  }
}

void era_joint_print_state(FILE* stream, era_joint_state_p state) {
  fprintf(stream, "%14s: % 8.2f deg\n",
    "shoulder_yaw", rad_to_deg(state->shoulder_yaw));
  fprintf(stream, "%14s: % 8.2f deg\n",
    "shoulder_roll", rad_to_deg(state->shoulder_roll));
  fprintf(stream, "%14s: % 8.2f deg\n",
    "shoulder_pitch", rad_to_deg(state->shoulder_pitch));
  fprintf(stream, "%14s: % 8.2f deg\n",
    "elbow_pitch", rad_to_deg(state->elbow_pitch));
  fprintf(stream, "%14s: % 8.2f deg\n",
    "tool_roll", rad_to_deg(state->tool_roll));
  fprintf(stream, "%14s: % 8.2f deg\n",
    "tool_opening", rad_to_deg(state->tool_opening));
}

void era_joint_print_trajectory(FILE* stream, era_joint_trajectory_p
  trajectory) {
  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s  %12s\n",
    "shoulder_yaw",
    "shoulder_roll",
    "shoulder_pitch",
    "elbow_pitch",
    "tool_roll",
    "tool_opening",
    "limit_error");

  int i;
  for (i = 0; i < trajectory->num_points; i++) {
    fprintf(stream,
      "%12.2f deg  %12.2f deg  %12.2f deg  %12.2f deg  %12.2f deg  "
      "%12.2f deg  %12d\n",
      rad_to_deg(trajectory->points[i].shoulder_yaw),
      rad_to_deg(trajectory->points[i].shoulder_roll),
      rad_to_deg(trajectory->points[i].shoulder_pitch),
      rad_to_deg(trajectory->points[i].elbow_pitch),
      rad_to_deg(trajectory->points[i].tool_roll),
      rad_to_deg(trajectory->points[i].tool_opening),
      trajectory->limit_errors[i]);
  }
}

int era_joint_read_trajectory(const char* filename, era_joint_trajectory_p
  trajectory) {
  int i, result;
  FILE* file;
  char buffer[1024];

  era_joint_state_t point;
  double timestamp;

  era_joint_init_trajectory(trajectory, 0);

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

      trajectory->points = realloc(trajectory->points,
        (trajectory->num_points+1)*sizeof(era_joint_state_t));
      trajectory->points[trajectory->num_points] = point;

      if (result == 7) {
        trajectory->timestamps = realloc(trajectory->timestamps,
          (trajectory->num_points+1)*sizeof(double));
        trajectory->timestamps[trajectory->num_points] = timestamp;
      }

      ++trajectory->num_points;
    }
  }

  fclose(file);

  return trajectory->num_points;
}

int era_joint_write_trajectory(const char* filename, era_joint_trajectory_p
  trajectory) {
  int i;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");

  if (file == NULL)
    return -ERA_JOINT_ERROR_FILE_CREATE;

  for (i = 0; i < trajectory->num_points; ++i) {
    sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
      trajectory->points[i].shoulder_yaw,
      trajectory->points[i].shoulder_roll,
      trajectory->points[i].shoulder_pitch,
      trajectory->points[i].elbow_pitch,
      trajectory->points[i].tool_roll,
      trajectory->points[i].tool_opening,
      trajectory->timestamps[i]);

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_JOINT_ERROR_FILE_WRITE;
    }
  }

  fclose(file);

  return trajectory->num_points;
}
