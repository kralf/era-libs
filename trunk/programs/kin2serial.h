/*	Header-file for 
 *	Connecting
 *      Kinematic system model for BlueBotics ERA-5/1
 *      with libserial
 *
 * 	Fritz Stöckli   stfritz@ethz.ch
 * 	Last change:    3.5.2007
 */


#include"kin.h"


void target_init_starting_values(t_target* target);
void theta_init_start_tiks(t_theta* th);

void theta_tiks_to_rad(t_theta* th);
void theta_rad_to_tiks(t_theta* th);

void kin2s_init(t_theta* th, t_target* target);

void target_user_read(t_target* target);

void kin2s_position_mode(t_target* target);


/* print to screen */
void theta_print_rad(t_theta* th);
void theta_print_tiks(t_theta* th);
void target_print(t_target* th);
