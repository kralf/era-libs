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
  PDEBUG_SNIP("%d\n",x);
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

void set_homing_method(int id, char method)
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

/* end new functions ----------------------------------------------------------------- */


