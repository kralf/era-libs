/*	Trayectory generation for BlueBotics ERA-5/1
 *
 * 	Fritz Stoeckli   stfritz@ethz.ch
 * 	Last change:     6.5.2008
 */

#include <stdlib.h>

#include "io.h"
#include "errors.h"

int era_read_arm_trajectory(
  const char *filename,
  era_arm_configuration_t** arm_trajectory,
  double** timestamps) {
  int i, result;
  int num_configurations = 0;
  FILE* file;
  char buffer[1024];

  era_arm_configuration_t arm_configuration;
  double timestamp;

  *arm_trajectory = 0;
  *timestamps = 0;

  file = fopen(filename, "r");

  if (file == NULL) return -ERA_ERROR_FILE_OPEN;

  while (fgets(buffer, sizeof(buffer), file) != NULL) {
    if (buffer[0] != '#') {
      result = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf",
        &arm_configuration.shoulder_yaw,
        &arm_configuration.shoulder_roll,
        &arm_configuration.shoulder_pitch,
        &arm_configuration.ellbow_pitch,
        &arm_configuration.tool_roll,
        &arm_configuration.tool_opening,
        &timestamp);

      if (result < 6) {
        fclose(file);
        return -ERA_ERROR_FILE_FORMAT;
      }

      *arm_trajectory = realloc(*arm_trajectory,
        (num_configurations+1)*sizeof(era_arm_configuration_t));
      (*arm_trajectory)[num_configurations] = arm_configuration;

      if (result == 7) {
        *timestamps = realloc(*timestamps,
          (num_configurations+1)*sizeof(double));
        (*timestamps)[num_configurations] = timestamp;
      }

      num_configurations++;
    }
  }

  fclose(file);
  return num_configurations;
}

int era_write_arm_trajectory(
  const char *filename,
  const era_arm_configuration_t* arm_trajectory,
  const double* timestamps,
  int num_configurations) {
  int i, result;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");

  if (file == NULL) return -ERA_ERROR_FILE_CREATE;

  for (i = 0; i < num_configurations; i++) {
    result = sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f",
      arm_trajectory[i].shoulder_yaw,
      arm_trajectory[i].shoulder_roll,
      arm_trajectory[i].shoulder_pitch,
      arm_trajectory[i].ellbow_pitch,
      arm_trajectory[i].tool_roll,
      arm_trajectory[i].tool_opening);

    if (timestamps)
      result += sprintf(&buffer[result], " %.6f\n", timestamps[i]);
    else
      result += sprintf(&buffer[result], "\n");

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_ERROR_FILE_WRITE;
    }
  }

  fclose(file);
  return num_configurations;
}

int era_read_tool_trajectory(
  const char *filename,
  era_tool_configuration_t** tool_trajectory,
  double** timestamps) {
  int i, result;
  int num_configurations = 0;
  FILE* file;
  char buffer[1024];

  era_tool_configuration_t tool_configuration;
  double timestamp;

  *tool_trajectory = 0;
  *timestamps = 0;

  file = fopen(filename, "r");

  if (file == NULL) return -ERA_ERROR_FILE_OPEN;

  while (fgets(buffer, sizeof(buffer), file) != NULL) {
    if (buffer[0] != '#') {
      result = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf",
        &tool_configuration.x,
        &tool_configuration.y,
        &tool_configuration.z,
        &tool_configuration.yaw,
        &tool_configuration.roll,
        &tool_configuration.opening,
        &timestamp);

      if (result < 6) {
        fclose(file);
        return -ERA_ERROR_FILE_FORMAT;
      }

      *tool_trajectory = realloc(*tool_trajectory,
        (num_configurations+1)*sizeof(era_tool_configuration_t));
      (*tool_trajectory)[num_configurations] = tool_configuration;

      if (result == 7) {
        *timestamps = realloc(*timestamps,
          (num_configurations+1)*sizeof(double));
        (*timestamps)[num_configurations] = timestamp;
      }

      num_configurations++;
    }
  }

  fclose(file);
  return num_configurations;
}

int era_write_tool_trajectory(
  const char *filename,
  const era_tool_configuration_t* tool_trajectory,
  const double* timestamps,
  int num_configurations) {
  int i, result;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");

  if (file == NULL) return -ERA_ERROR_FILE_CREATE;

  for (i = 0; i < num_configurations; i++) {
    result = sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f",
      tool_trajectory[i].x,
      tool_trajectory[i].y,
      tool_trajectory[i].z,
      tool_trajectory[i].yaw,
      tool_trajectory[i].roll,
      tool_trajectory[i].opening);

    if (timestamps)
      result += sprintf(&buffer[result], " %.6f\n", timestamps[i]);
    else
      result += sprintf(&buffer[result], "\n");

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_ERROR_FILE_WRITE;
    }
  }

  fclose(file);
  return num_configurations;
}

int era_read_velocity_profile(
  const char *filename,
  era_arm_velocity_t** arm_velocities,
  double** timestamps) {
  int i, result;
  int num_velocities = 0;
  FILE* file;
  char buffer[1024];

  era_arm_velocity_t arm_velocity;
  double timestamp;

  *arm_velocities = 0;
  *timestamps = 0;

  file = fopen(filename, "r");

  if (file == NULL) return -ERA_ERROR_FILE_OPEN;

  while (fgets(buffer, sizeof(buffer), file) != NULL) {
    if (buffer[0] != '#') {
      result = sscanf(buffer, "%lf %lf %lf %lf %lf %lf %lf",
        &arm_velocity.shoulder_yaw,
        &arm_velocity.shoulder_roll,
        &arm_velocity.shoulder_pitch,
        &arm_velocity.ellbow_pitch,
        &arm_velocity.tool_roll,
        &arm_velocity.tool_opening,
        &timestamp);

      if (result < 7) {
        fclose(file);
        return -ERA_ERROR_FILE_FORMAT;
      }

      *arm_velocities = realloc(*arm_velocities,
        (num_velocities+1)*sizeof(era_arm_velocity_t));
      (*arm_velocities)[num_velocities] = arm_velocity;
      *timestamps = realloc(*timestamps, (num_velocities+1)*sizeof(double));
      (*timestamps)[num_velocities] = timestamp;

      num_velocities++;
    }
  }

  fclose(file);
  return num_velocities;
}

int era_write_velocity_profile(
  const char *filename,
  const era_arm_velocity_t* arm_velocities,
  const double* timestamps,
  int num_velocities) {
  int i, result;
  FILE* file;
  char buffer[1024];

  file = fopen(filename, "w");

  if (file == NULL) return -ERA_ERROR_FILE_CREATE;

  for (i = 0; i < num_velocities; i++) {
    result = sprintf(buffer, "%.6f %.6f %.6f %.6f %.6f %.6f\n",
      arm_velocities[i].shoulder_yaw,
      arm_velocities[i].shoulder_roll,
      arm_velocities[i].shoulder_pitch,
      arm_velocities[i].ellbow_pitch,
      arm_velocities[i].tool_roll,
      arm_velocities[i].tool_opening,
      timestamps[i]);

    if (fputs(buffer, file) <= 0) {
      fclose(file);
      return -ERA_ERROR_FILE_WRITE;
    }
  }

  fclose(file);
  return num_velocities;
}
