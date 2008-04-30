#ifndef _LIB_EPOS_H
#define _LIB_EPOS_H

#define MAXERRORHISTORY 30

#include "non-asl/cpc.h"
#include "non-asl/cpclib.h"
#include "libepos_type.h"

ALL_EPOS_READ myepos_read;
ALL_EPOS_SET myepos_set;	/* not yet used in libepos */



/* *************************** */
/* INIT / CONTROL OPERATIONS   */
/* *************************** */
void switch_on(int id); 				/* turns the motor to SWITCH_ON state */
void enable_quick_stop(int id);
void fault_reset(int id);
void enable_operation(int id);				/* enables the motor operation */
void activate(int id);
void activate_position(int id);
void shutdown(int id); 					/* turns the motor to READY_TO_SWITCH_ON state */
void disable_voltage(int id); 				/* turns the motor to SWITCH_ON_DISABLED state */
void quick_stop(int id); 				/* turns the motor to QUICK_STOP_ACTIV state*/

/* begin new functions --------------------------------------------------------------- */
void start_homing_operation(int id);
/* end new functions ----------------------------------------------------------------- */


/* *************************** */
/* ***** SET OPERATIONS ****** */
/* *************************** */
void set_mode_of_operation(int id, int mode); 		/* sets the control mode */
void set_profile_acceleration(int id, long int a);
void set_profile_deceleration(int id, long int a);
void set_profile_velocity(int id, long int v); 
void set_position_window(int id, unsigned int a);
void set_position_window_time(int id,long int a);
void set_motion_profile_type(int id, long int a);
void set_maximum_following_error(int id, unsigned int a);
void set_maximal_profile_velocity(int id, long int v);
void set_quick_stop_deceleration(int id, long int v);
void set_current_value(int id, short current);
void set_position_control_parameter_set(int id, long int p_gain, long int i_gain, long int d_gain, long int v_feedward, long int a_feedforward);
void set_velocity_control_parameter_set(int id, long int p_gain, long int i_gain);
void set_target_velocity(int id, long int v);
void set_target_position(int id, long int x);
/* begin new functions --------------------------------------------------------------- */
void set_velocity_mode_setting_value(int id, long int v);
void set_position_mode_setting_value(int id, long int x);
void set_controlword(int id, int val);
void set_home_offset(int id, long int x);
void set_homing_speed_switch_search(int id, long int v);
void set_homing_speed_zero_search(int id, long int v);
void set_homing_method(int id, int method);
void set_software_minimal_position_limit(int id, long int x);
void set_software_maximal_position_limit(int id, long int x);
void set_continous_current_limit(int id, int i);
void set_output_current_limit(int id, int i);
void set_homing_current_threshold(int id, int i);
void set_RS232_baudrate(int id, int val);

/* end new functions ----------------------------------------------------------------- */



/* *************************** */
/* ***** READ OPERATIONS ***** */
/* *************************** */
/* message handler */
void read_SDO_msg_handler(int handle, const CPC_MSG_T * cpcmsg);

void get_mode_of_operation(int id); 			/* sets the control mode */
void get_mode_of_operation_display(int id);
void get_position_control_parameter_set(int id);
void get_velocity_control_parameter_set(int id);
/* position */
void get_actual_position(int id);
/* velocity */
void get_actual_velocity(int id);
void get_averaged_velocity(int id);
void get_maximum_following_error(int id);
/* current */
void get_current_value(int id);				/* commanding value */
void get_current_actual_value(int id);			/* measured value */
void get_current_actual_value_averaged(int id);

void get_maximum_profile_velocity(int id);


/* begin new functions --------------------------------------------------------------- */
void get_velocity_mode_setting_value(int id);
void get_position_mode_setting_value(int id);
void get_statusword(int id);
void get_controlword(int id);
void get_home_offset(int id);
void get_homing_speed_switch_search(int id);
void get_homing_speed_zero_search(int id);
void get_homing_method(int id);
void get_software_version(int id);
void get_software_minimal_position_limit(int id);
void get_software_maximal_position_limit(int id);
void get_profile_acceleration(int id);
void get_profile_deceleration(int id);
void get_error_register(int id);
void get_error_history(int id, int index);
void get_error(int id);
void get_actual_error_register(int id);
void get_actual_error_history(int id);
void get_continous_current_limit(int id);
void get_output_current_limit(int id);
/* end new functions ----------------------------------------------------------------- */

#endif






