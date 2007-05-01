#include <stdio.h>
#include "libepos.h"
#include "libcan.h"
#include "pdebug.h"

/* *************************** */
/* INIT / CONTROL OPERATIONS   */
/* *************************** */
/* turns the motor to READY_TO_SWITCH_ON state */
void shutdown(int id)
{
  PDEBUG("requesting shutdown\n"); 
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;	//0x2b
  msg[1]= 0x40;	
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x06;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg); 
}

/* turns the motor to SWITCH_ON_DISABLED state */
void quick_stop(int id)
{
  PDEBUG("requesting disable voltage\n"); 
  char msg[7];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x02;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg); 
}

/* FAULT_RESET */
void faultreset(int id)
{
  PDEBUG("requesting fault reset\n"); 
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0xFF;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg); 
}
/* turns the motor to SWITCH_ON state */
void switch_on(int id)
{
  PDEBUG("requesting switch on\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x07;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg); 
}

/* enables the motor operation */
void enable_operation(int id)
{
  PDEBUG("activating...\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x0F;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void enable_quick_stop(int id)
{
  PDEBUG("requesting quick stop\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x0F;
  msg[5]= 0x01;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

/* activate new velocity demand */
void activate(int id)
{
  enable_operation(id);
}

/* activate new position demand */
void activate_position(int id)
/* sending control word suiting position mode
   bit 4: Assume target position
   bit 5: execute now
   bit 6: 0:abs value */
{
  PDEBUG("activate position demand\n ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x3f;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

/* turns the motor to SWITCH_ON_DISABLED state */
void disable_voltage(int id)
{
  PDEBUG("requesting disable voltage\n"); 
  char msg[7];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg); 
}

/* turns the motor to SWITCH_ON_DISABLED state */
void fault_reset(int id)
{
  PDEBUG("requesting fault reset\n"); 
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x80;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg); 
}

void start_homing_operation(int id)
{
  PDEBUG("start homing operation...\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x1F;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}


/* *************************** */
/* ***** SET OPERATIONS ****** */
/* *************************** */

/* sets the control mode */
void set_mode_of_operation(int id, int mode)
{
  PDEBUG("set mode of operation to 0x%x\n",mode);
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_1_BYTE;
  msg[1]= 0x60;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= mode;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void set_target_velocity(int id, long int v)
{
  //PDEBUG("[libepos] set target velocity to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0xff;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (v & 0x000000ff);
  msg[5]= ((v & 0x0000ff00)>>8);
  msg[6]= ((v & 0x00ff0000)>>16);
  msg[7]= ((v & 0xff000000)>>24);
  //PDEBUG_SNIP("%x (%i)\n",v,v);
  my_send_can_message(can_id, msg);  
}

void set_target_position(int id, long int x)
{
  //PDEBUG("set target position to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x7a;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (x & 0x000000ff);
  msg[5]= ((x & 0x0000ff00)>>8);
  msg[6]= ((x & 0x00ff0000)>>16);
  msg[7]= ((x & 0xff000000)>>24);
  //PDEBUG_SNIP("%d\n",x);
  my_send_can_message(can_id, msg);  
}

void set_profile_velocity(int id,long int v)
{
  PDEBUG("set profile velocity to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x81;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (v & 0x000000ff);
  msg[5]= ((v & 0x0000ff00)>>8);
  msg[6]= ((v & 0x00ff0000)>>16);
  msg[7]= ((v & 0xff000000)>>24);
  PDEBUG_SNIP("%ld\n",v);
  my_send_can_message(can_id, msg);  
}

void set_profile_acceleration(int id,long int a)
{
  PDEBUG("set profile acceleration to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x83;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (a & 0x000000ff);
  msg[5]= ((a & 0x0000ff00)>>8);
  msg[6]= ((a & 0x00ff0000)>>16);
  msg[7]= ((a & 0xff000000)>>24);
  PDEBUG_SNIP("%lx\n",a);
  my_send_can_message(can_id, msg);  
}  

void set_profile_deceleration(int id, long int a)
{
  PDEBUG("set profile deceleration to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x84;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (a & 0x000000ff);
  msg[5]= ((a & 0x0000ff00)>>8);
  msg[6]= ((a & 0x00ff0000)>>16);
  msg[7]= ((a & 0xff000000)>>24);
  PDEBUG_SNIP("%lx\n",a);
  my_send_can_message(can_id, msg);  
}
  
void set_position_window(int id, unsigned int a)
{
  PDEBUG("set position window to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x67;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (a & 0x000000ff);
  msg[5]= ((a & 0x0000ff00)>>8);
  msg[6]= ((a & 0x00ff0000)>>16);
  msg[7]= ((a & 0xff000000)>>24);
  PDEBUG_SNIP("%x\n",a);
  my_send_can_message(can_id, msg);  
}  

void set_position_window_time(int id,long int a)
{
  PDEBUG("set position window time to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x68;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (a & 0x000000ff);
  msg[5]= ((a & 0x0000ff00)>>8);
  msg[6]= 0;
  msg[7]= 0;
  PDEBUG_SNIP("%lx\n",a);
  my_send_can_message(can_id, msg);  
} 

void set_maximum_following_error(int id, unsigned int a)
{
  PDEBUG("set maximum following error to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x65;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (a & 0x000000ff);
  msg[5]= ((a & 0x0000ff00)>>8);
  msg[6]= ((a & 0x00ff0000)>>16);
  msg[7]= ((a & 0xff000000)>>24);
  PDEBUG_SNIP("%x\n",a);
  my_send_can_message(can_id, msg);  
}  

void set_motion_profile_type(int id, long int a)
{
  PDEBUG("set motion profile type to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x86;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (a & 0x000000ff);
  msg[5]= ((a & 0x0000ff00)>>8);
  msg[6]= 0;
  msg[7]= 0;
  PDEBUG_SNIP("%lx\n",a);
  my_send_can_message(can_id, msg);  
} 

void set_motor_data(int id, EPOS_MOTOR_DATA_STR motor_data)
{
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x01;
  msg[4]= (motor_data.continuous_current_limit & 0x000000ff);
  msg[5]= ((motor_data.continuous_current_limit & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();

  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x02;
  msg[4]= (motor_data.output_current_limit & 0x000000ff);
  msg[5]= ((motor_data.output_current_limit & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();
  
  msg[0]= WRITE_1_BYTE;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x03;
  msg[4]= (motor_data.pole_pair_number & 0x000000ff);
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();
  
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x04;
  msg[4]= (motor_data.max_speed_in_current_mode & 0x000000ff);
  msg[5]= ((motor_data.max_speed_in_current_mode & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();
  
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x04;
  msg[4]= (motor_data.thermal_time_constant_winding & 0x000000ff);
  msg[5]= ((motor_data.thermal_time_constant_winding & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
 
}

void set_position_control_parameter_set(int id, long int p_gain, long int i_gain, long int d_gain, long int v_feedforward, long int a_feedforward)
{ 
  PDEBUG("set position control parameter set\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x01;
  msg[4]= (p_gain & 0x000000ff);
  msg[5]= ((p_gain & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  PDEBUG("p_gain: %lx\n",p_gain);
  read_can_message();
  
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x02;
  msg[4]= (i_gain & 0x000000ff);
  msg[5]= ((i_gain & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  PDEBUG("i_gain: %lx\n",i_gain);
  read_can_message();
  
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x03;
  msg[4]= (d_gain & 0x000000ff);
  msg[5]= ((d_gain & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  PDEBUG("d_gain %lx\n",d_gain);
  read_can_message();
    
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x04;  
  msg[4]= (v_feedforward & 0x000000ff);
  msg[5]= ((v_feedforward & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  PDEBUG("v_feedforward %lx\n",v_feedforward);
  read_can_message();

  msg[0]= WRITE_2_BYTE;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x05;
  msg[4]= (a_feedforward & 0x000000ff);
  msg[5]= ((a_feedforward & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  PDEBUG("a_feedforward %lx\n",a_feedforward);
 
}

void set_velocity_control_parameter_set(int id, long int p_gain, long int i_gain)
{ 
  PDEBUG("set velocity control parameter set\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0xF9;
  msg[2]= 0x60;
  msg[3]= 0x01;
  msg[4]= (p_gain & 0x000000ff);
  msg[5]= ((p_gain & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  PDEBUG("p_gain: %lx\n",p_gain);
  read_can_message();
  
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0xF9;
  msg[2]= 0x60;
  msg[3]= 0x02;
  msg[4]= (i_gain & 0x000000ff);
  msg[5]= ((i_gain & 0x0000ff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  PDEBUG("i_gain: %lx\n",i_gain);
  read_can_message();
}

void set_maximal_profile_velocity(int id, long int v)
{
  //PDEBUG("[libepos] set maximal profile velocity to ");
  char msg[7];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x7f;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (v & 0x000000ff);
  msg[5]= ((v & 0x0000ff00)>>8);
  msg[6]= ((v & 0x00ff0000)>>16);
  msg[7]= ((v & 0xff000000)>>24);
  //PDEBUG_SNIP("%lx (%i)\n",v,v);
  my_send_can_message(can_id, msg);
}

void set_quick_stop_deceleration(int id, long int v)
{
  PDEBUG("set quick strop deceleration to ");
  char msg[7];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x85;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (v & 0x000000ff);
  msg[5]= ((v & 0x0000ff00)>>8);
  msg[6]= ((v & 0x00ff0000)>>16);
  msg[7]= ((v & 0xff000000)>>24);
  PDEBUG_SNIP("%lx\n",v);
  my_send_can_message(can_id, msg);
}

void set_current_value(int id, short current)
{
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x30;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= (current & 0x00ff);
  msg[5]= ((current & 0xff00)>>8);
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

/* begin new functions --------------------------------------------------------------- */

void set_velocity_mode_setting_value(int id, long int v)
{
  PDEBUG("set velocity mode setting value to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x6B;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= (v & 0x000000ff);
  msg[5]= ((v & 0x0000ff00)>>8);
  msg[6]= ((v & 0x00ff0000)>>16);
  msg[7]= ((v & 0xff000000)>>24);
  PDEBUG_SNIP("%d\n",v);
  my_send_can_message(can_id, msg);  
}

void set_position_mode_setting_value(int id, long int x)
{
  PDEBUG("set position mode setting value to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x62;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= (x & 0x000000ff);
  msg[5]= ((x & 0x0000ff00)>>8);
  msg[6]= ((x & 0x00ff0000)>>16);
  msg[7]= ((x & 0xff000000)>>24);
  PDEBUG_SNIP("%ld\n",x);
  my_send_can_message(can_id, msg);  
}

void set_controlword(int id, int val)
{
  PDEBUG("set controlword to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (val & 0x000000ff);
  msg[5]= ((val & 0x0000ff00)>>8);
  msg[6]= 0;
  msg[7]= 0;
  PDEBUG_SNIP("%x\n",val);
  my_send_can_message(can_id, msg);  
} 

void set_home_offset(int id, long int x)
{
  PDEBUG("set home offset position to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x7C;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= (x & 0x000000ff);
  msg[5]= ((x & 0x0000ff00)>>8);
  msg[6]= ((x & 0x00ff0000)>>16);
  msg[7]= ((x & 0xff000000)>>24);
  PDEBUG_SNIP("%ld\n",x);
  my_send_can_message(can_id, msg);  
}

void set_homing_speed_switch_search(int id, long int v)
{
  PDEBUG("set homing speed switch search to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x99;
  msg[2]= 0x60;
  msg[3]= 0x01;
  msg[4]= (v & 0x000000ff);
  msg[5]= ((v & 0x0000ff00)>>8);
  msg[6]= ((v & 0x00ff0000)>>16);
  msg[7]= ((v & 0xff000000)>>24);
  PDEBUG_SNIP("%ld\n",v);
  my_send_can_message(can_id, msg);  
}

void set_homing_speed_zero_search(int id, long int v)
{
  PDEBUG("set homing speed zero search to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x99;
  msg[2]= 0x60;
  msg[3]= 0x02;
  msg[4]= (v & 0x000000ff);
  msg[5]= ((v & 0x0000ff00)>>8);
  msg[6]= ((v & 0x00ff0000)>>16);
  msg[7]= ((v & 0xff000000)>>24);
  PDEBUG_SNIP("%ld\n",v);
  my_send_can_message(can_id, msg);  
}

void set_homing_method(int id, int method)
{
  PDEBUG("set homing method to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_1_BYTE;
  msg[1]= 0x98;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= method;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  PDEBUG_SNIP("%d\n",x);
  my_send_can_message(can_id, msg);  
}

void set_software_minimal_position_limit(int id, long int x)
{
  PDEBUG("set software minimal position limit to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x7D;
  msg[2]= 0x60;
  msg[3]= 0x01;
  msg[4]= (x & 0x000000ff);
  msg[5]= ((x & 0x0000ff00)>>8);
  msg[6]= ((x & 0x00ff0000)>>16);
  msg[7]= ((x & 0xff000000)>>24);
  PDEBUG_SNIP("%ld\n",x);
  my_send_can_message(can_id, msg);  
}

void set_software_maximal_position_limit(int id, long int x)
{
  PDEBUG("set software maximal position limit to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_4_BYTE;
  msg[1]= 0x7D;
  msg[2]= 0x60;
  msg[3]= 0x02;
  msg[4]= (x & 0x000000ff);
  msg[5]= ((x & 0x0000ff00)>>8);
  msg[6]= ((x & 0x00ff0000)>>16);
  msg[7]= ((x & 0xff000000)>>24);
  PDEBUG_SNIP("%ld\n",x);
  my_send_can_message(can_id, msg);  
}

void set_continous_current_limit(int id, int i)
{
  PDEBUG("set continous current limit to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x01;
  msg[4]= (i & 0x000000ff);
  msg[5]= ((i & 0x0000ff00)>>8);
  msg[6]= 0;
  msg[7]= 0;
  PDEBUG_SNIP("%x\n",val);
  my_send_can_message(can_id, msg);  
} 

void set_output_current_limit(int id, int i)
{
  PDEBUG("set output current limit to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x02;
  msg[4]= (i & 0x000000ff);
  msg[5]= ((i & 0x0000ff00)>>8);
  msg[6]= 0;
  msg[7]= 0;
  PDEBUG_SNIP("%x\n",val);
  my_send_can_message(can_id, msg);  
} 

void set_homing_current_threshold(int id, int i)
{
  PDEBUG("set homing current threshold to ");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= WRITE_2_BYTE;
  msg[1]= 0x80;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= (i & 0x000000ff);
  msg[5]= ((i & 0x0000ff00)>>8);
  msg[6]= 0;
  msg[7]= 0;
  PDEBUG_SNIP("%x\n",val);
  my_send_can_message(can_id, msg);  
} 





/* end new functions ----------------------------------------------------------------- */

/* *************************** */
/* ***** READ OPERATIONS ***** */
/* *************************** */

/* gets the control mode */
void get_mode_of_operation(int id)
{
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x60;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_velocity_control_parameter_set(int id)
{ 
  PDEBUG("ask for velocity control parameter set\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0xF9;
  msg[2]= 0x60;
  msg[3]= 0x01;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();
  
  msg[0]= READ;
  msg[1]= 0xF9;
  msg[2]= 0x60;
  msg[3]= 0x02;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_actual_position(int id)
{
  //PDEBUG("ask for actual position\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x64;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_actual_velocity(int id)
{ 
  //PDEBUG("ask for actual velocity\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x6C;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_actual_velocity_avg(int id)
{ 
  //PDEBUG("ask for averaged velocity\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x28;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_maximum_following_error(int id)
{ 
  PDEBUG("ask maximum following error\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x65;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_motion_profile_type(int id)
{ 
  //PDEBUG("ask for motion profile type\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x86;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_motor_data(int id)
{ 
  //PDEBUG("ask for motor data \n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x01;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();

  msg[0]= READ;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x02;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();

  msg[0]= READ;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x03;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();

  msg[0]= READ;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x04;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();

  msg[0]= READ;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x05;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_position_control_parameter_set(int id)
{ 
  //PDEBUG("ask for position control parameter set\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x01;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();
  
  msg[0]= READ;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x02;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();
  
  msg[0]= READ;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x03;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();
    
  msg[0]= READ;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x04;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
  read_can_message();

  msg[0]= READ;
  msg[1]= 0xFB;
  msg[2]= 0x60;
  msg[3]= 0x05;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);

}

void get_current_actual_value(int id)
{ 
  //PDEBUG("ask for actual current value (%i)\n", id);
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x78;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}


void get_current_actual_value_averaged(int id)
{ 
  //PDEBUG("ask for actual current value averaged\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x27;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_mode_of_operation_display(int id)
{
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x61;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_current_value(int id)
{
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x30;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);
}

void get_maximum_profile_velocity(int id)
{
  //PDEBUG("ask for actual position\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x7F;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

/* begin new functions --------------------------------------------------------------- */

void get_velocity_mode_setting_value(int id)
{
  PDEBUG("ask for actual velocity mode setting value\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x6B;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_position_mode_setting_value(int id)
{
  PDEBUG("ask for actual position mode setting value\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x62;
  msg[2]= 0x20;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_statusword(int id)
{
  PDEBUG("ask for actual statusword\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x41;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_controlword(int id)
{
  PDEBUG("ask for actual controlword\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x40;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_home_offset(int id)
{
  PDEBUG("ask for actual home offset\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x7C;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_homing_speed_switch_search(int id)
{
  PDEBUG("ask for homing speed switch search\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x99;
  msg[2]= 0x60;
  msg[3]= 0x01;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_homing_speed_zero_search(int id)
{
  PDEBUG("ask for homing speed zero search\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x99;
  msg[2]= 0x60;
  msg[3]= 0x02;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_homing_method(int id)
{
  PDEBUG("ask for homing method\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x98;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_software_version(int id)
{
  //PDEBUG("ask for actual software version\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x03;
  msg[2]= 0x20;
  msg[3]= 0x01;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_software_minimal_position_limit(int id)
{
  PDEBUG("ask for actual software minimal position limit\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x7D;
  msg[2]= 0x60;
  msg[3]= 0x01;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_software_maximal_position_limit(int id)
{
  PDEBUG("ask for actual software maximal position limit\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x7D;
  msg[2]= 0x60;
  msg[3]= 0x02;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_profile_acceleration(int id)
{
  PDEBUG("ask for actual profile acceleration\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x83;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_profile_deceleration(int id)
{
  PDEBUG("ask for actual profile deceleration\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x84;
  msg[2]= 0x60;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_actual_error_register(int id)
{
  PDEBUG("ask for actual error register\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x01;
  msg[2]= 0x10;
  msg[3]= 0x00;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_actual_error_history(int id)
{
  PDEBUG("ask for actual error history\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x03;
  msg[2]= 0x10;
  msg[3]= 0x01;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_continous_current_limit(int id)
{
  PDEBUG("ask for actual continous current limit\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x01;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}

void get_output_current_limit(int id)
{
  PDEBUG("ask for actual output current limit\n");
  char msg[8];
  int can_id = 0x600+id;
  msg[0]= READ;
  msg[1]= 0x10;
  msg[2]= 0x64;
  msg[3]= 0x02;
  msg[4]= 0x00;
  msg[5]= 0x00;
  msg[6]= 0x00;
  msg[7]= 0x00;
  my_send_can_message(can_id, msg);  
}



/* end new functions ----------------------------------------------------------------- */



/* ------------------------------------------------------------------
   Incomming Messages
   ----------------------------------------------------------------*/
void read_SDO_msg_handler(int handle, const CPC_MSG_T * cpcmsg)
{ 
  
  PDEBUG("mhm... something to read... ");
  PDEBUG_SNIP("id: %lx Data: ",cpcmsg->msg.canmsg.id);
  PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[0]);
  PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[1]);
  PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[2]);
  PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[3]);
  PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[4]);
  PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[5]);
  PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[6]);
  PDEBUG_SNIP("%x \n",cpcmsg->msg.canmsg.msg[7]);

  if ((cpcmsg->msg.canmsg.id > 0x80)&& (cpcmsg->msg.canmsg.id < 0x100))
    {
		//EMERGERNCY object
		PDEBUG("ERROR ERROR ERROR ERROR ERROR ERROR ERROR ERROR\n");
		int errorfound=0;
		if ((cpcmsg->msg.canmsg.msg[0]==0x00)&&(cpcmsg->msg.canmsg.msg[1]==0x10))
		{
			PDEBUG("==> generic error <==\n"); 
			errorfound=1;
		}
		if ((cpcmsg->msg.canmsg.msg[0]==0x10)&&(cpcmsg->msg.canmsg.msg[1]==0x23))
		{
			PDEBUG("==> over current error <==\n"); 
			errorfound=1;
		}
		if ((cpcmsg->msg.canmsg.msg[0]==0x20)&&(cpcmsg->msg.canmsg.msg[1]==0x32))
		{
			PDEBUG("==> under voltage error <==\n"); 
			errorfound=1;
		}
		if ((cpcmsg->msg.canmsg.msg[0]==0x10)&&(cpcmsg->msg.canmsg.msg[1]==0x42))
		{
			PDEBUG("==> over temperature error <==\n"); 
			errorfound=1;
		} 
		if ((cpcmsg->msg.canmsg.msg[0]==0x13)&&(cpcmsg->msg.canmsg.msg[1]==0x51))
		{
			PDEBUG("==> supply voltage too low error <==\n"); 
			errorfound=1;
		} 
		if ((cpcmsg->msg.canmsg.msg[0]==0x00)&&(cpcmsg->msg.canmsg.msg[1]==0x61))
		{
			PDEBUG("==> internal software error <==\n"); 
			errorfound=1;
		}
		if ((cpcmsg->msg.canmsg.msg[0]==0x00)&&(cpcmsg->msg.canmsg.msg[1]==0x61))
		{
			PDEBUG("==> internal software error <==\n"); 
			errorfound=1;
		} 
			if ((cpcmsg->msg.canmsg.msg[0]==0x11)&&(cpcmsg->msg.canmsg.msg[1]==0x86))
		{
			PDEBUG("==> Following error <==\n");
			errorfound=1;
		}
		if (errorfound==0)
			PDEBUG("==> Error not specified in libepos.c <==");
		
		faultreset(1);
		activate_position(1);
		faultreset(1);
		activate(1);
		faultreset(1);
		//shutdown(1);
		//faultreset(1);
		//shutdown(1);
    }

  if (cpcmsg->msg.canmsg.msg[0] == 0x80) //We have an error message --> print for debug
  { //TODO: add id to be shure message is not for LSS
      PDEBUG("mhm... configuration error... ");
      PDEBUG_SNIP("id: %lx Data: ",cpcmsg->msg.canmsg.id);
      PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[0]);
      PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[1]);
      PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[2]);
      PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[3]);
      PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[4]);
      PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[5]);
      PDEBUG_SNIP("%x ",cpcmsg->msg.canmsg.msg[6]);
      PDEBUG_SNIP("%x \n",cpcmsg->msg.canmsg.msg[7]);
  }
  else
  {
	
	if ((cpcmsg->msg.canmsg.id >= 0x581)&&(cpcmsg->msg.canmsg.id <= (0x581+NUMBER_OF_EPOS-1)))
	{

 	  if ((cpcmsg->msg.canmsg.msg[1]==0x7F) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("[libepos] received maximum profile velocity value ");
	      //PDEBUG_SNIP("byte 0: %x value: ",cpcmsg->msg.canmsg.msg[0]);
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].max_profile_velocity
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8)
		+(cpcmsg->msg.canmsg.msg[6]<<16)
		+(cpcmsg->msg.canmsg.msg[7]<<24);
	      //PDEBUG("0x%x (%i)\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].max_profile_velocity, myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].max_profile_velocity);
	    }

	  if ((cpcmsg->msg.canmsg.msg[1]==0x64) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("received position actual value ");
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].actual_position //myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)]
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8)
		+(cpcmsg->msg.canmsg.msg[6]<<16)
		+(cpcmsg->msg.canmsg.msg[7]<<24);

	      //PDEBUG_SNIP("ox%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].actual_position);
	    }
	  if ((cpcmsg->msg.canmsg.msg[1]==0x69) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("received velocity sensor actual value ");
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].sensed_velocity
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8)
		+(cpcmsg->msg.canmsg.msg[6]<<16)
		+(cpcmsg->msg.canmsg.msg[7]<<24);
	      //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].sensed_velocity);
	    }
	  
	  if ((cpcmsg->msg.canmsg.msg[1]==0x6C) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("received velocity actual value \n");
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].actual_velocity
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8)
		+(cpcmsg->msg.canmsg.msg[6]<<16)
		+(cpcmsg->msg.canmsg.msg[7]<<24);
	    }
	  if ((cpcmsg->msg.canmsg.msg[1]==0x28) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x20) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("received velocity averaged value ");
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].actual_velocity
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8)
		+(cpcmsg->msg.canmsg.msg[6]<<16)
		+(cpcmsg->msg.canmsg.msg[7]<<24);
	      //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].actual_velocity_avg);
	    }
	  
	  if ((cpcmsg->msg.canmsg.msg[1]==0x67) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("received position window value ");
	      //PDEBUG_SNIP("byte 0: %x value: ",cpcmsg->msg.canmsg.msg[0]);
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_window
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8)
		+(cpcmsg->msg.canmsg.msg[6]<<16)
		+(cpcmsg->msg.canmsg.msg[7]<<24);
	      //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_window);
	    }
  
	  if ((cpcmsg->msg.canmsg.msg[1]==0x65) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("received maximum following error value ");
	      //PDEBUG_SNIP("byte 0: %x value: ",cpcmsg->msg.canmsg.msg[0]);
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_window
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8)
		+(cpcmsg->msg.canmsg.msg[6]<<16)
		+(cpcmsg->msg.canmsg.msg[7]<<24);
	      //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].maximum_following_error);
	    }
	  if ((cpcmsg->msg.canmsg.msg[1]==0x68) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("received position window time value "); 
	      //PDEBUG_SNIP("byte 0: %x value: ",cpcmsg->msg.canmsg.msg[0]);
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_window_time
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8); 
	      //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_window_time);
	    }
	  if ((cpcmsg->msg.canmsg.msg[1]==0x86) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
	      //PDEBUG("received motion profile type "); 
	      //PDEBUG_SNIP("byte 0: %x value: ",cpcmsg->msg.canmsg.msg[0]);
	      myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motion_profile_type
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8); 
	      //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motion_profile_type);
	    }

 	  if ((cpcmsg->msg.canmsg.msg[1]==0x78) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
		myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].actual_current
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8);		
	    }

	  if ((cpcmsg->msg.canmsg.msg[1]==0x27) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x20) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {

		myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].actual_current_avg
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8);
	    }

	  if ((cpcmsg->msg.canmsg.msg[1]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
		myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].operation_mode
		=cpcmsg->msg.canmsg.msg[4];		
	    }

	  if ((cpcmsg->msg.canmsg.msg[1]==0x61) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
		myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].operation_mode_display
		=cpcmsg->msg.canmsg.msg[4];		
	    }

	  if ((cpcmsg->msg.canmsg.msg[1]==0x30) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x20) 
	      && (cpcmsg->msg.canmsg.msg[3]==0x00))
	    {
		myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].current_value
		=cpcmsg->msg.canmsg.msg[4]
		+(cpcmsg->msg.canmsg.msg[5]<<8);
	    }


	  if ((cpcmsg->msg.canmsg.msg[1]==0xFB) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60))
	    {
	      //PDEBUG("received position config "); 
	      if (cpcmsg->msg.canmsg.msg[3]==0x01)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.p_gain
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.p_gain);
		}
	  
	      if (cpcmsg->msg.canmsg.msg[3]==0x02)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.i_gain
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.i_gain);
		}
	      if (cpcmsg->msg.canmsg.msg[3]==0x03)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.d_gain
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24);
		  //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.d_gain);
		}
	      if (cpcmsg->msg.canmsg.msg[3]==0x04)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.v_feedforward
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.v_feedforward);
		}
	      
	      if (cpcmsg->msg.canmsg.msg[3]==0x05)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.a_feedforward
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].position_config.a_feedforward);
		}
	      
	    }

	   if ((cpcmsg->msg.canmsg.msg[1]==0xF9) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x60))
	    {
	      //PDEBUG("received velocity config "); 
	      if (cpcmsg->msg.canmsg.msg[3]==0x01)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].velocity_config.p_gain
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].velocity_config.p_gain);
		}
	  
	      if (cpcmsg->msg.canmsg.msg[3]==0x02)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].velocity_config.i_gain
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].velocity_config.i_gain);
		}
	    }

	  if ((cpcmsg->msg.canmsg.msg[1]==0x10) 
	      && (cpcmsg->msg.canmsg.msg[2]==0x64))
	    {
	      //PDEBUG("got motor data ");
	      if (cpcmsg->msg.canmsg.msg[3]==0x01)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.continuous_current_limit
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("Continuous current limit: 0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.continuous_current_limit);
		}
	      if (cpcmsg->msg.canmsg.msg[3]==0x02)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.output_current_limit
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("Output current limit: 0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.output_current_limit);
		}
	        if (cpcmsg->msg.canmsg.msg[3]==0x03)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.pole_pair_number
		    =cpcmsg->msg.canmsg.msg[4]
		    +(0<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("Pole Pair Number: 0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.pole_pair_number);
		}
		if (cpcmsg->msg.canmsg.msg[3]==0x04)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.max_speed_in_current_mode
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("Max speed in current mode: 0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.max_speed_in_current_mode);
		}
		if (cpcmsg->msg.canmsg.msg[3]==0x05)
		{
		  myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.thermal_time_constant_winding
		    =cpcmsg->msg.canmsg.msg[4]
		    +(cpcmsg->msg.canmsg.msg[5]<<8)
		    +(0<<16)
		    +(0<<24); 
		  //PDEBUG_SNIP("Thermal time constant winding: 0x%x\n",myepos_read.number[(cpcmsg->msg.canmsg.id - 0x581)].motor_data.thermal_time_constant_winding);
		}
	    } 

	}
    }
}


  
