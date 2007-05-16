#ifndef _LIB_EPOS_TYPE_H
#define _LIB_EPOS_TYPE_H

//#define OPERATION_MODE_PROFILE_VELOCITY 0x3
//#define OPERATION_MODE_PROFILE_POSITION 0x1
//#define OPERATION_MODE_CURRENT 0xFD

#define WRITE_1_BYTE 0x2f
#define WRITE_2_BYTE 0x2b
#define WRITE_4_BYTE 0x23
#define READ 0x40

#define NUMBER_OF_EPOS 6

typedef struct EPOS_VELOCITY {
	unsigned short int current_epos;
	int velocity;
} EPOS_VELOCITY;

typedef struct EPOS_POSITION_CONFIG_STR {
  int p_gain;
  int i_gain;
  int d_gain;
  int v_feedforward;
  int a_feedforward;
} EPOS_POSITION_CONFIG_STR;

typedef struct EPOS_VELOCITY_CONFIG_STR {
  int p_gain;
  int i_gain;
} EPOS_VELOCITY_CONFIG_STR;

typedef struct EPOS_MOTOR_DATA_STR {
  unsigned int continuous_current_limit;
  unsigned int output_current_limit;
  unsigned int pole_pair_number;
  unsigned int max_speed_in_current_mode;
  unsigned int thermal_time_constant_winding;
} EPOS_MOTOR_DATA_STR;


typedef struct EPOS_ERROR_HISTORY{
	int code;
	unsigned char reg;
	char *msg;
	
} EPOS_ERROR_HISTORY;

typedef struct EPOS_ERROR_DEVICE{
	int count;
	unsigned char reg;
	EPOS_ERROR_HISTORY history[5];
} EPOS_ERROR_DEVICE;

typedef struct EPOS_ERROR_SERIAL{
	long int code;
	char *msg;
} EPOS_ERROR_SERIAL;

typedef struct EPOS_ERROR {
	EPOS_ERROR_DEVICE device;	
	EPOS_ERROR_SERIAL serial;
} EPOS_ERROR;

///////////////////////////////////////

/* EPOS READ */
typedef struct EPOS_READ{
  int sensed_velocity;
  int motion_profile_type;
  int actual_velocity;
  int actual_velocity_avg;
  short actual_current;
  short actual_current_avg;
  int actual_position;
  char operation_mode;
  char operation_mode_display;
  short current_value;

  unsigned int maximum_following_error;
  EPOS_MOTOR_DATA_STR motor_data;

  int position_window_time;
  unsigned int position_window;

  EPOS_VELOCITY_CONFIG_STR velocity_config;	/*velocity mode configuration*/
  EPOS_POSITION_CONFIG_STR position_config;	/*position mode configuration*/

  int max_profile_velocity;
	
  EPOS_ERROR error;		/* Epos errors */	

}EPOS_READ;


typedef struct ALL_EPOS_READ {
  EPOS_READ number[NUMBER_OF_EPOS];
} ALL_EPOS_READ;


/* EPOS SET */
typedef struct EPOS_SET{
  int des_velocity;		/*desired velocity*/
  int des_maxvelocity;		/*desired maximum velocity*/
  int des_acceleration;		/*desired acceleration*/
  int des_deceleration;		/*desired deceleration*/
  int des_stopdeceleration;	/*desired emergency stop deceleration*/
  int des_position;		/*desired position*/
  EPOS_VELOCITY_CONFIG_STR velocity_config;	/*velocity mode configuration*/
  EPOS_POSITION_CONFIG_STR position_config;	/*position mode configuration*/

  /* input values (args) */
  int log_flag;					/* flag indicates data logging on/off */
}EPOS_SET;


typedef struct ALL_EPOS_SET {
  EPOS_SET number[NUMBER_OF_EPOS];
} ALL_EPOS_SET;

#endif
