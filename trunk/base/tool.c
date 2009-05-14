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

void era_tool_init_trajectory(era_tool_trajectory_p trajectory,
  ssize_t num_points) {
  int i;

  trajectory->num_points = num_points;

  if (num_points) {
    trajectory->points = malloc(num_points*sizeof(era_tool_trajectory_t));
    trajectory->timestamps = malloc(num_points*sizeof(double));

    for (i = 0; i < num_points; ++i) {
      era_tool_init_state(&trajectory->points[i]);
      trajectory->timestamps[i] = 0.0;
    }
  }
  else {
    trajectory->points = 0;
    trajectory->timestamps = 0;
  }
}

void era_tool_destroy_trajectory(era_tool_trajectory_p trajectory) {
  if (trajectory->num_points) {
    free(trajectory->points);
    free(trajectory->timestamps);

    trajectory->num_points = 0;
    trajectory->points = 0;
    trajectory->timestamps = 0;
  }
}

void era_tool_print_state(FILE* stream, era_tool_state_p state) {
  fprintf(stream, "%7s: % 8.4f m\n",
    "x", state->x);
  fprintf(stream, "%7s: % 8.4f m\n",
    "y", state->y);
  fprintf(stream, "%7s: % 8.4f m\n",
    "z", state->z);
  fprintf(stream, "%7s: % 8.2f °\n",
    "yaw", rad_to_deg(state->yaw));
  fprintf(stream, "%7s: % 8.2f °\n",
    "roll", rad_to_deg(state->roll));
  fprintf(stream, "%7s: % 8.2f °\n",
    "opening", rad_to_deg(state->opening));
}

void era_tool_print_trajectory(FILE* stream, era_tool_trajectory_p
  trajectory) {
  fprintf(stream, "%14s  %14s  %14s  %14s  %14s  %14s\n",
    "x",
    "y",
    "z",
    "yaw",
    "roll",
    "opening");

  int i;
  for (i = 0; i < trajectory->num_points; i++) {
    fprintf(stream,
      "%12.4f m  %12.4f m  %12.4f m  %12.2f °  %12.2f °  %12.2f °\n",
      trajectory->points[i].x,
      trajectory->points[i].y,
      trajectory->points[i].z,
      rad_to_deg(trajectory->points[i].yaw),
      rad_to_deg(trajectory->points[i].roll),
      rad_to_deg(trajectory->points[i].opening));
  }
}

int era_tool_read_trajectory(const char* filename, era_tool_trajectory_p
  trajectory) {
  int i, result;
  FILE* file;
  char buffer[1024];

  era_tool_state_t point;
  double timestamp;

  era_tool_init_trajectory(trajectory, 0);

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

      trajectory->points = realloc(trajectory->points,
        (trajectory->num_points+1)*sizeof(era_tool_state_t));
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

int era_tool_write_trajectory(const char* filename, era_tool_trajectory_p
  trajectory) {
  int i;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");

  if (file == NULL)
    return -ERA_TOOL_ERROR_FILE_CREATE;

  for (i = 0; i < trajectory->num_points; ++i) {
    sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f %.6f\n",
      trajectory->points[i].x,
      trajectory->points[i].y,
      trajectory->points[i].z,
      trajectory->points[i].yaw,
      trajectory->points[i].roll,
      trajectory->points[i].opening,
      trajectory->timestamps[i]);

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_TOOL_ERROR_FILE_WRITE;
    }
  }

  fclose(file);

  return trajectory->num_points;
}
