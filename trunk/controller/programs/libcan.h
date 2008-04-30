
void canHWInit(void);
void my_send_can_message(int can_id, char *msg);
void read_SDO_msg_handler(int handle, const CPC_MSG_T * cpcmsg);
void read_can_message(void);

