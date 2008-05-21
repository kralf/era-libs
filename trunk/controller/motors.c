/*      Interfacing kinematic system model for BlueBotics ERA-5/1
 *      with EPOS control library
 *
 *      Fritz Stoeckli   stfritz@ethz.ch
 *      Last change:     7.5.2008
 */

#include <stdlib.h>
#include <math.h>

#include <epos.h>

#include "motors.h"

#define ERA_MOTORS_POLL_INTERVAL 100

#define sqr(x) x*x
#define max(x, y) x>=y?x:y

const era_motor_configuration_t era_motor_tiks_per_revolution = {
  .shoulder_yaw = 2000,
  .shoulder_roll = 2000,
  .shoulder_pitch = 2000,
  .ellbow_pitch = 2000,
  .tool_roll = 2000,
  .tool_opening = 2000,
};

const era_motor_configuration_t era_motor_current_limit = {
  .shoulder_yaw = 2500,
  .shoulder_roll = 2000,
  .shoulder_pitch = 2000,
  .ellbow_pitch = 2000,
  .tool_roll = 300,
  .tool_opening = 300,
};

const era_motor_configuration_t era_motor_zero = {
  .shoulder_yaw = 80000,
  .shoulder_roll = 99000,
  .shoulder_pitch = 50000,
  .ellbow_pitch = 0,
  .tool_roll = 145000,
  .tool_opening = 30000,
};

const era_motor_configuration_t era_motor_home_sensors = {
  .shoulder_yaw = 45592,
  .shoulder_roll = 42443,
  .shoulder_pitch = 42722,
  .ellbow_pitch = 35608,
  .tool_roll = 182580,
  .tool_opening = 36300,
};

const era_motor_configuration_t era_motor_home_current = {
  .shoulder_yaw = 50000,
  .shoulder_roll = 50000,
  .shoulder_pitch = 50000,
  .ellbow_pitch = 50000,
  .tool_roll = 180000,
  .tool_opening = 30000,
};

const era_motor_configuration_t era_motor_homing_method_sensors = {
  .shoulder_yaw = EPOS_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH,
  .shoulder_roll = EPOS_HOMING_METHOD_POSITIVE_LIMIT_SWITCH,
  .shoulder_pitch = EPOS_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH,
  .ellbow_pitch = EPOS_HOMING_METHOD_NEGATIVE_LIMIT_SWITCH,
  .tool_roll = EPOS_HOMING_METHOD_POSITIVE_LIMIT_SWITCH,
  .tool_opening = EPOS_HOMING_METHOD_POSITIVE_CURRENT_THRESHOLD,
};

const era_motor_configuration_t era_motor_homing_method_current = {
  .shoulder_yaw = EPOS_HOMING_METHOD_NEGATIVE_CURRENT_THRESHOLD,
  .shoulder_roll = EPOS_HOMING_METHOD_POSITIVE_CURRENT_THRESHOLD,
  .shoulder_pitch = EPOS_HOMING_METHOD_NEGATIVE_CURRENT_THRESHOLD,
  .ellbow_pitch = EPOS_HOMING_METHOD_NEGATIVE_CURRENT_THRESHOLD,
  .tool_roll = EPOS_HOMING_METHOD_POSITIVE_CURRENT_THRESHOLD,
  .tool_opening = EPOS_HOMING_METHOD_POSITIVE_CURRENT_THRESHOLD,
};

const era_motor_configuration_t era_motor_home_current_threshold = {
  .shoulder_yaw = 1700,
  .shoulder_roll = 1200,
  .shoulder_pitch = 1400,
  .ellbow_pitch = 1400,
  .tool_roll = 150,
  .tool_opening = 150,
};

const era_motor_velocity_t era_motor_homing_velocity = {
  .shoulder_yaw = 200,
  .shoulder_roll = 200,
  .shoulder_pitch = 200,
  .ellbow_pitch = 200,
  .tool_roll = 500,
  .tool_opening = 200,
};

const era_arm_configuration_t era_motor_gear_transmission = {
  .shoulder_yaw = -0.02*0.108695652,
  .shoulder_roll = -0.02*0.119047619,
  .shoulder_pitch = 0.02*0.119047619,
  .ellbow_pitch = 0.02*0.129032258,
  .tool_roll = 0.005*1.0,
  .tool_opening = 0.01*1.0,
};

static era_motor_configuration_t era_motor_home = {
  .shoulder_yaw = 0,
  .shoulder_roll = 0,
  .shoulder_pitch = 0,
  .ellbow_pitch = 0,
  .tool_roll = 0,
  .tool_opening = 0,
};

void era_print_motor_configuration(
  FILE* stream,
  const era_motor_configuration_t* motor_configuration) {
  fprintf(stream, "shoulder_yaw:   %d tiks\n",
    motor_configuration->shoulder_yaw);
  fprintf(stream, "shoulder_roll:  %d tiks\n",
    motor_configuration->shoulder_roll);
  fprintf(stream, "shoulder_pitch: %d tiks\n",
    motor_configuration->shoulder_pitch);
  fprintf(stream, "ellbow_pitch:   %d tiks\n",
    motor_configuration->ellbow_pitch);
  fprintf(stream, "tool_roll:      %d tiks\n",
    motor_configuration->tool_roll);
  fprintf(stream, "tool_opening:   %d tiks\n",
    motor_configuration->tool_opening);
}

void era_print_motor_velocity(
  FILE* stream,
  const era_motor_velocity_t* motor_velocity) {
  fprintf(stream, "shoulder_yaw:   %d rpm\n",
    motor_velocity->shoulder_yaw);
  fprintf(stream, "shoulder_roll:  %d rpm\n",
    motor_velocity->shoulder_roll);
  fprintf(stream, "shoulder_pitch: %d rpm\n",
    motor_velocity->shoulder_pitch);
  fprintf(stream, "ellbow_pitch:   %d rpm\n",
    motor_velocity->ellbow_pitch);
  fprintf(stream, "tool_roll:      %d rpm\n",
    motor_velocity->tool_roll);
  fprintf(stream, "tool_opening:   %d rpm\n",
    motor_velocity->tool_opening);
}

void era_motor_configuration_init(
  era_motor_configuration_t* motor_configuration) {
  motor_configuration->shoulder_yaw = 0;
  motor_configuration->shoulder_roll = 0;
  motor_configuration->shoulder_pitch = 0;
  motor_configuration->ellbow_pitch = 0;
  motor_configuration->tool_roll = 0;
  motor_configuration->tool_opening = 0;
}

void era_arm_to_motor(
  const era_arm_configuration_t* arm_configuration,
  const era_arm_velocity_t* arm_velocity,
  era_motor_configuration_t* motor_configuration,
  era_motor_velocity_t* motor_velocity) {
  int i;

  double* arm_conf = (double*)arm_configuration;
  int* motor_conf = (int*)motor_configuration;
  double* arm_vel = (double*)arm_velocity;
  int* motor_vel = (int*)motor_velocity;

  int* zero = (int*)&era_motor_zero;
  int* home = (int*)&era_motor_home;
  int* tiks_per_revolution = (int*)&era_motor_tiks_per_revolution;
  double* gear = (double*)&era_motor_gear_transmission;

  for (i = 0; i < 6; i++) {
    if (arm_conf && motor_conf)
      motor_conf[i] = arm_conf[i]/gear[i]*tiks_per_revolution[i]/(2*M_PI)+
      zero[i]-home[i];

    if (arm_vel && motor_vel)
      motor_vel[i] = arm_vel[i]/gear[i]*60/(2*M_PI);
  }
}

void era_motor_to_arm(
  const era_motor_configuration_t* motor_configuration,
  const era_motor_velocity_t* motor_velocity,
  era_arm_configuration_t* arm_configuration,
  era_arm_velocity_t* arm_velocity) {
  int i;

  int* motor_conf = (int*)motor_configuration;
  double* arm_conf = (double*)arm_configuration;
  int* motor_vel = (int*)motor_velocity;
  double* arm_vel = (double*)arm_velocity;

  int* zero = (int*)&era_motor_zero;
  int* home = (int*)&era_motor_home;
  int* tiks_per_revolution = (int*)&era_motor_tiks_per_revolution;
  double* gear = (double*)&era_motor_gear_transmission;

  for (i = 0; i < 6; i++) {
    if (motor_conf && arm_conf)
      arm_conf[i] = (double)(motor_conf[i]-zero[i]+home[i])*gear[i]*
      2*M_PI/tiks_per_revolution[i];

    if (arm_vel && motor_vel)
      arm_vel[i] = motor_vel[i]*gear[i]*2*M_PI/60;
  }
}

void era_motors_init(
  const char* dev) {
  int id;

  int* current_limit = (int*)&era_motor_current_limit;

  can_init(dev);

  for (id = 1; id < 7; id++) {
    epos_fault_reset(id);
    epos_set_output_current_limit(id, current_limit[id-1]);
  }
}

void era_motors_close(void) {
  int id;

  for (id = 1; id < 7; id++) epos_shutdown(id);

  can_close();
}

void era_motors_wait(
  int condition) {
  int id, status = 0;

  while (!status) {
    status = 0xFFFFFFFF;

    for (id = 1; id < 7; id++) {
      epos_get_statusword(id);
      status &= condition & epos_read.node[id-1].status;
    }

    if (!status) usleep(ERA_MOTORS_POLL_INTERVAL);
  }
}

int era_motors_set_mode(
  int operation_mode) {
  int id;

  if ((operation_mode != ERA_OPERATION_MODE_POSITION) &&
    (operation_mode != ERA_OPERATION_MODE_VELOCITY))
    return 0;

  for (id = 1; id < 7; id++) {
    epos_shutdown(id);
    epos_enable_operation(id);

    if (operation_mode == ERA_OPERATION_MODE_POSITION) {
      epos_set_mode_of_operation(id, EPOS_OPERATION_MODE_PROFILE_POSITION);
    }
    else if (operation_mode == ERA_OPERATION_MODE_VELOCITY) {
      epos_set_velocity_mode_setting_value(id, 0);
      epos_set_mode_of_operation(id, EPOS_OPERATION_MODE_VELOCITY);
    }

    epos_shutdown(id);
    epos_enable_operation(id);
  }

  return 1;
}

void era_motors_home(
  int homing_mode) {
  int id;

  if ((homing_mode != ERA_HOMING_MODE_SENSORS) &&
    (homing_mode != ERA_HOMING_MODE_CURRENT))
    return;

  int* motor_home = (int*)&era_motor_home;
  int* home;
  int* homing_method;
  int* current_threshold = (int*)&era_motor_home_current_threshold;
  int* velocity = (int*)&era_motor_homing_velocity;

  if (homing_mode == ERA_HOMING_MODE_SENSORS) {
    home = (int*)&era_motor_home_sensors;
    homing_method = (int*)&era_motor_homing_method_sensors;
  }
  else if (homing_mode == ERA_HOMING_MODE_CURRENT) {
    home = (int*)&era_motor_home_current;
    homing_method = (int*)&era_motor_homing_method_current;
  }

  for (id = 1; id < 7; id++) {
    epos_shutdown(id);
    epos_enable_operation(id);
    epos_set_mode_of_operation(id, EPOS_OPERATION_MODE_HOMING);
    epos_shutdown(id);
    epos_enable_operation(id);

    epos_set_homing_method(id, homing_method[id-1]);
    motor_home[id-1] = home[id-1];

    epos_set_home_offset(id, motor_home[id-1]);
    epos_set_homing_current_threshold(id, current_threshold[id-1]);
    epos_set_homing_speed_switch_search(id, velocity[id-1]);
    epos_set_homing_speed_zero_search(id, velocity[id-1]);

    epos_start_homing_operation(id);
  }
}

void era_position_mode_get(
  era_arm_configuration_t* arm_configuration) {
  int id;

  era_motor_configuration_t motor_configuration;
  int* motor = (int*)&motor_configuration;

  for (id = 1; id < 7; ++id) {
    epos_get_actual_position(id);
    motor[id-1] = epos_read.node[id-1].actual_position;
  }

  era_motor_to_arm(&motor_configuration, 0, arm_configuration, 0);
}

void era_position_mode_set(
  const era_arm_configuration_t* arm_configuration,
  const era_arm_velocity_t* arm_velocity) {
  int id;

  era_motor_configuration_t motor_configuration;
  era_motor_velocity_t motor_velocity;
  int* motor_conf = (int*)&motor_configuration;
  int* motor_vel = (int*)&motor_velocity;

  era_arm_to_motor(arm_configuration, arm_velocity, &motor_configuration,
    &motor_velocity);

  for (id = 1; id < 7; ++id) {
    epos_set_profile_velocity(id, abs(motor_vel[id-1]));
    epos_set_target_position(id, motor_conf[id-1]);
  }

  for (id = 1; id < 7; ++id) epos_activate_position(id);
}

void era_velocity_mode_zero(void) {
  int id;

  for (id = 1; id < 7; id++) epos_set_velocity_mode_setting_value(id, 0);
}

void era_velocity_mode_set(
  const era_arm_velocity_t* arm_velocity) {
  int id;

  era_motor_velocity_t motor_velocity;
  int* motor_vel = (int*)&motor_velocity;

  era_arm_to_motor(0, arm_velocity, 0, &motor_velocity);

  for (id = 1; id < 7; id++)
    epos_set_velocity_mode_setting_value(id, motor_vel[id-1]);
}

double era_position_error(
  const era_arm_configuration_t* arm_configuration) {
  int i;
  double squared_error = 0.0;

  double* arm_conf = (double*)arm_configuration;

  era_arm_configuration_t arm_current_configuration;
  double* current = (double*)&arm_current_configuration;

  era_position_mode_get(&arm_current_configuration);

  for (i = 0; i < 6; ++i) squared_error += sqr(arm_conf[i]-current[i]);

  return sqrt(squared_error);
}
