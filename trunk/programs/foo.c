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

/* end new functions ----------------------------------------------------------------- */