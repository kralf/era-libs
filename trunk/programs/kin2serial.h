/*	Header-file for 
 *	Connecting
 *      Kinematic system model for BlueBotics ERA-5/1
 *      with libserial
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    20.5.2007
 */


#include"kin.h"










void target_init_starting_values(float target[]);
void theta_init_start_tiks(float theta[]);

void theta_tiks_to_rad(float theta[]);
void theta_rad_to_tiks(float theta[]);

void kin2s_init(float theta[], float target[]);

void target_user_read(float target[]);

void kin2s_position_mode(float target[]);
void kin2s_position_mode_init();
void kin2s_position_mode_calc_vel(float pos[], float pos_old[], float vel[], float vel_max);
void kin2s_position_mode_set(float pos[], float vel[]);

float kin2s_position_error(float pos[]);

/* print to screen */
void theta_print_rad(float theta[]);
void theta_print_tiks(float theta[]);
void target_print(float target[]);
