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

#include "base/trajectory.h"

const char* era_trajectory_errors[] = {
  "success",
  "error opening file",
  "invalid file format",
  "error creating file",
  "error writing file",
  "state undefined",
};

void era_trajectory_init(era_trajectory_p trajectory) {
  int i;
  spline_p spline_a = (spline_p)trajectory;

  for (i = 0; i < sizeof(era_trajectory_t)/sizeof(spline_t); ++i)
    spline_init(&spline_a[i]);
}

void era_trajectory_destroy(era_trajectory_p trajectory) {
  int i;
  spline_p spline_a = (spline_p)trajectory;

  for (i = 0; i < sizeof(era_trajectory_t)/sizeof(spline_t); ++i)
    spline_destroy(&spline_a[i]);
}

int era_trajectory_read(const char* filename, era_trajectory_p trajectory) {
  int result;
  FILE* file;
  char buffer[16384];

  era_trajectory_init(trajectory);
  spline_p spline_a = (spline_p)trajectory;

  file = fopen(filename, "r");
  if (file == NULL)
    return -ERA_TRAJECTORY_ERROR_FILE_OPEN;

  while (fgets(buffer, sizeof(buffer), file) != NULL) {
    if (buffer[0] != '#') {
      spline_segment_t segment_a[sizeof(era_trajectory_t)/sizeof(spline_t)];
      double arg_width;

      int i, num_chars;
      char* buffer_pos = buffer;
      for (i = 0; i < sizeof(era_trajectory_t)/sizeof(spline_t); ++i) {
        result = sscanf(buffer_pos, "%lg %lg %lg %lg %n",
          &segment_a[i].a,
          &segment_a[i].b,
          &segment_a[i].c,
          &segment_a[i].d,
          &num_chars);
  
        if (result < 4) {
          fclose(file);
          return -ERA_TRAJECTORY_ERROR_FILE_FORMAT;
        }
        else
          buffer_pos += num_chars;
      }

      result = sscanf(buffer_pos, "%lg", &arg_width);

      if (result < 1) {
        fclose(file);
        return -ERA_TRAJECTORY_ERROR_FILE_FORMAT;
      }

      for (i = 0; i < sizeof(era_trajectory_t)/sizeof(spline_t); ++i) {
        segment_a[i].arg_width = arg_width;
        spline_add_segment(&spline_a[i], &segment_a[i]);
      }
    }
  }

  fclose(file);

  return spline_a[0].num_segments;
}

int era_trajectory_write(const char* filename, era_trajectory_p trajectory) {
  int i;
  FILE* file;
  char buffer[16384];

  spline_p spline_a = (spline_p)trajectory;

  file = fopen(filename, "w");
  if (file == NULL)
    return -ERA_TRAJECTORY_ERROR_FILE_CREATE;

  for (i = 0; i < spline_a[0].num_segments; ++i) {
    int j, num_chars;
    char* buffer_pos = buffer;
    for (j = 0; j < sizeof(era_trajectory_t)/sizeof(spline_t); ++j) {
      num_chars = sprintf(buffer_pos, "%lg %lg %lg %lg \n",
        spline_a[j].segments[i].a,
        spline_a[j].segments[i].b,
        spline_a[j].segments[i].c,
        spline_a[j].segments[i].d);

      buffer_pos += num_chars;
    }
    sprintf(buffer_pos, "%lg\n", spline_a[0].segments[i].arg_width);

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_TRAJECTORY_ERROR_FILE_WRITE;
    }
  }

  fclose(file);

  return spline_a[0].num_segments;
}

int era_trajectory_evaluate(era_trajectory_p trajectory, double time,
  int seg_index, era_joint_state_p joint_state, era_velocity_state_p vel_state,
  era_acceleration_state_p accel_state) {
  int i;
  spline_p spline_a = (spline_p)trajectory;
  double* joint_state_a = (double*)joint_state;
  double* vel_state_a = (double*)vel_state;
  double* accel_state_a = (double*)accel_state;

  for (i = 0; i < sizeof(era_trajectory_t)/sizeof(spline_t); ++i) {
    if (joint_state_a) 
      seg_index = spline_evaluate_linear_search(&spline_a[i], 
        spline_eval_type_base_function, time, seg_index, &joint_state_a[i]);
    if (vel_state_a) 
      seg_index = spline_evaluate_linear_search(&spline_a[i], 
        spline_eval_type_first_derivative, time, seg_index, &vel_state_a[i]);
    if (accel_state_a) 
      seg_index = spline_evaluate_linear_search(&spline_a[i], 
        spline_eval_type_second_derivative, time, seg_index,
        &accel_state_a[i]);

    if (seg_index < 0)
      return -ERA_TRAJECTORY_ERROR_UNDEFINED;
  }
  
  return seg_index;
}
