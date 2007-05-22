/*	Header-file for serial EPOS-communication
 *	V0.4
 * 	(C) Marc Rauer ETHZ	marc.rauer(at)gmx.de
 * 	Last change: 05/16/07
 */


/**
*  	\file
*	\brief
*	EPOS-communication over RS232
*  <h2>Introduction</h2>
*
*  This layer provides low-level mechanisms for communicating with EPOS
*  motion controllers over a serial connection
*/


#include "libepos.h"

/* defines: */
/**	Value of the acknowledge byte, if dataframe was valid.
*/
#define OKAY 		0x4F
/** Value of the acknowledge byte, if dataframe was invalid.
*/
#define FAILED		0x46
/** Value of the op-code for a response message.
*/
#define RESPONSE	0x00

/** Default baudrate using EPOS.
*/
#define BAUDRATE B38400
/** Current serial device for communication.
*/
#define MODEMDEVICE "/dev/ttyS0"
/** Default timeout [seconds] during reading form serial device (used by select()).
*/
#define TIMEOUTSEC 0
/** Default timeout [nanoseconds] during reading form serial device (used by select()).
*/
#define TIMEOUTNSEC 500000 /* 500ms */
/** Specifies maximum retries during failed read/write-operations.
*/
#define MAXRETRY 10
/** Current entries in the error-structure for handling specific serial errors.
*/
#define MAXERRORSERIAL 31

/** Defines a datatype of size word for handling the CRC-algorithm.
*/
typedef unsigned short word;



/* function-prototypes: */

/**
*	Initialize the serial port
*
*	\note
*	Same function name in libserial.c than in libcan.c to easy switch from CAN to RS232 using libepos.c
*/


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

/** Read bytes from device.
*
*	\note
*	Uses select() for non-infinit blocking (see TIMEOUTSEC).
*
*  	\return
*
*    	- 0 : timeout reading a byte
*    	- 0 : number of read bytes
*    	- -1 : unspecified error in read()
*/
int read_byte(/** File descriptor */ int fd,
				/** Array containing read bytes */ char *buffer);


int write_byte(int, char);
int write_string(int, char *, int);
//int c2w(char *, word *, int);
int w2c(word *, char *, int);

/** Change order of databytes in the dataframe.
*
*	\return
*	Number of changed bytes within the dataframe.
*/
int chg_byte_order(/** Array of bytes for which change order*/ char *data,
					/** Number of bytes in the array (length)*/ int no_chars);

/** Change order of datawords in the dataframe.
*
*	\return
*	Number of changed bytes within the dataframe.
*/
int chg_word_order(/** Array of words for which change order*/ char *data,
					/** Number of bytes! in the array (length)*/ int no_chars);


int calc_crc(char *, char *, int);

void prtmsg(char *, char *, int);
int ClearIOBuffer(int);

