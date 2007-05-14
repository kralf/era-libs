/*	Header-file for serial EPOS-communication
 *	V0.4
 * 	� Marc Rauer ETHZ	marc.rauer@gmx.de
 * 	Last change: 04/21/07
 */

#include "libepos.h"
 
/* defines: */
#define OKAY 		0x4F
#define FAILED		0x46
#define RESPONSE	0x00

#define BAUDRATE B38400
#define MODEMDEVICE "/dev/ttyS0"
#define TIMEOUTSEC 0
#define TIMEOUTNSEC 500000 /* 500ms */
#define MAXRETRY 10
#define MAXERRORSERIAL 31


typedef unsigned short word; 






/* function-prototypes: */

void canHWInit();
void canHWEnd();

void my_send_can_message(int can_id, char *msg);
void read_can_message();

int epos2serial(int, char *, char *);
void serial2epos(int can_id, char *data_send, char *data_recv);
int handle_serial_errorcode(char *data_recv);

int open_device();
int close_device(int);
int receive_dataframe(int, char *);
int send_dataframe(int, char *, int);

int read_byte(int, char *);
int write_byte(int, char);
int write_string(int, char *, int);
int c2w(char *, word *, int);
int w2c(word *, char *, int);
int chg_byte_order(char *, int);
int chg_word_order(char *, int);
int calc_crc(char *, char *, int);

void prtmsg(char *, char *, int);
int ClearIOBuffer(int);

