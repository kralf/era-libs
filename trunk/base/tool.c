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

#include "tool.h"

#include "global.h"

const char* era_tool_errors[] = {
  "success",
  "error opening file",
  "invalid file format",
  "error creating file",
  "error writing file",
};

void era_tool_init_state(era_tool_state_p state) {
  memset(state, 0, sizeof(era_tool_state_t));
}

void era_tool_init_path(era_tool_path_p path, ssize_t num_points) {
  int i;

  path->num_points = num_points;

  if (num_points) {
    path->points = malloc(num_points*sizeof(era_tool_path_t));
    path->timestamps = malloc(num_points*sizeof(double));

    for (i = 0; i < num_points; ++i) {
      era_tool_init_state(&path->points[i]);
      path->timestamps[i] = 0.0;
    }
  }
  else {
    path->points = 0;
    path->timestamps = 0;
  }
}

void era_tool_destroy_path(era_tool_path_p path) {
  if (path->num_points) {
    free(path->points);
    free(path->timestamps);

    path->num_points = 0;
    path->points = 0;
    path->timestamps = 0;
  }
}

void era_tool_print_state(FILE* stream, era_tool_state_p state) {
  fprintf(stream, "%7s: %8.4f m\n",
    "x", state->x);
  fprintf(stream, "%7s: %8.4f m\n",
    "y", state->y);
  fprintf(stream, "%7s: %8.4f m\n",
    "z", state->z);
  fprintf(stream, "%7s: %8.2f deg\n",
    "yaw", rad_to_deg(state->yaw));
  fprintf(stream, "%7s: %8.2f deg\n",
    "roll", rad_to_deg(state->roll));
  fprintf(stream, "%7s: %8.2f deg\n",
    "opening", rad_to_deg(state->opening));
}

void era_tool_print_path(FILE* stream, era_tool_path_p path) {
  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s\n",
    "x",
    "y",
    "z",
    "yaw",
    "roll",
    "opening");

  int i;
  for (i = 0; i < path->num_points; i++) {
    fprintf(stream,
      "%12.4f m  %12.4f m  %12.4f m  %12.2f deg  %12.2f deg  %12.2f deg\n",
      path->points[i].x,
      path->points[i].y,
      path->points[i].z,
      rad_to_deg(path->points[i].yaw),
      rad_to_deg(path->points[i].roll),
      rad_to_deg(path->points[i].opening));
  }
}

int era_tool_read_path(const char* filename, era_tool_path_p path) {
  int i, result;
  FILE* file;
  char buffer[1024];

  era_tool_state_t point;
  double timestamp;

  era_tool_init_path(path, 0);

  file = fopen(filename, "r");
  if (file == NULL)
    return -ERA_TOOL_ERROR_FILE_OPEN;

  while (fgets(buffer, sizeof(buffer), file) != NULL) {
    if (buffer[0] != '#') {
      result = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf",
        &point.x,
        &point.y,
        &point.z,
        &point.yaw,
        &point.roll,
        &point.opening,
        &timestamp);

      if (result < 6) {
        fclose(file);
        return -ERA_TOOL_ERROR_FILE_FORMAT;
      }

      path->points = realloc(path->points,
        (path->num_points+1)*sizeof(era_tool_state_t));
      path->points[path->num_points] = point;

      if (result == 7) {
        path->timestamps = realloc(path->timestamps,
          (path->num_points+1)*sizeof(double));
        path->timestamps[path->num_points] = timestamp;
      }

      ++path->num_points;
    }
  }

  fclose(file);

  return path->num_points;
}

int era_tool_write_path(const char* filename, era_tool_path_p path) {
  int i;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");
  if (file == NULL)
    return -ERA_TOOL_ERROR_FILE_CREATE;

  for (i = 0; i < path->num_points; ++i) {
    sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
      path->points[i].x,
      path->points[i].y,
      path->points[i].z,
      path->points[i].yaw,
      path->points[i].roll,
      path->points[i].opening,
      path->timestamps[i]);

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_TOOL_ERROR_FILE_WRITE;
    }
  }

  fclose(file);

  return path->num_points;
}
