/*
 * This file is part of the software for Smartease, the mobile robot
 * platform used for the student robot contest at the Swiss Federal
 * Institute of Technology Lausanne (EPFL), Switzerland.
 * 
 * Copyright (C) 2002 Swiss Federal Institute of Technology, Lausanne.
 *                    All rights reserved.
 *
 * Developed at the Autonomous Systems Lab.
 * Visit our homepage at <http://asl.epfl.ch/>
 * 
 * Author: Roland Philippsen <roland dot philippsen at gmx dot net>
 * 
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */



#include "i2c.h"



/** \file
 *  \brief
 *  Basic I2C communication layer implementation.
 * 
 *  Compiles into a relocatable object file, part of libsmartease.so.
 */




#include <stdio.h>
#include <fcntl.h>
#include <string.h>

#ifndef WIN32
# include <unistd.h>
# include <termios.h>
#endif

#include "pdebug.h"



handle_t i2c_open_serial(const char *device,
			 unsigned long baud,
			 unsigned long timeoutmiliseconde)
{
#ifdef WIN32
	DCB  dcb;
	BOOL fSuccess;
	COMMTIMEOUTS timeout;
	handle_t handle;

	handle = CreateFile( device,	// Pointer to the name of the port
                      GENERIC_READ | GENERIC_WRITE,
                                    // Access (read-write) mode
                      0,            // Share mode
                      NULL,         // Pointer to the security attribute
                      OPEN_EXISTING,// How to open the serial port
                      0,            // Port attributes
                      NULL			// Handle to port with attribute
                      );            // to copy

	if (handle == INVALID_HANDLE_VALUE)	// Test if handle ok (Handle the error)
	{
		fprintf (stderr, "CreateFile failed with error &d.\n", GetLastError());
		return (-1);
	}
	// Build on the current configuration, and skip setting the size
	// if the input and output buffers with SetupComm

	fSuccess = GetCommState(handle, &dcb);

	if (!fSuccess)					// Handle the error
	{
		fprintf(stderr, "GetCommState failed with error %d.\n", GetLastError());
		return(-2);
	}

	// Fill in DCB: 19200 bps, 8 data bits, no parity, and 1 stop bit

	dcb.BaudRate = CBR_19200;		// Set the baud rate
	dcb.ByteSize = 8;				// data size, xmit, and rcv
	dcb.Parity = NOPARITY;			// no parity bit
	dcb.StopBits = ONESTOPBIT;		// one stop bit

	fSuccess = SetCommState(handle, &dcb);

	if(!fSuccess)					// Handle the error
	{
		fprintf(stderr, "SetCommState failed with error %d.\n", GetLastError());
		return(-3);
	}

	fSuccess = GetCommTimeouts(handle,&timeout);
	timeout.ReadIntervalTimeout=timeoutmiliseconde;
	timeout.ReadTotalTimeoutConstant=timeoutmiliseconde;
	timeout.ReadTotalTimeoutMultiplier=0;
	timeout.WriteTotalTimeoutMultiplier=0;
	timeout.WriteTotalTimeoutConstant=timeoutmiliseconde;
	SetCommTimeouts(handle,&timeout);
	PDEBUG("Serial port %s successfully reconfigured.\n", device);
	return(handle);


#else 
  handle_t fd;
  struct termios tio;

  fd = open(device, O_RDWR | O_NDELAY);

  if(fd == -1){
    fprintf(stderr, "i2c_open_serial: ");
    perror(device);
    return -1;
  }

  if(memset(&tio, 0, sizeof(struct termios)) != &tio){
    perror("i2c_open_serial: memset");
    close(fd);
    return -2;
  }
  
  switch(baud){
  case 50L    : tio.c_cflag = B50;     break;
  case 75L    : tio.c_cflag = B75;     break;
  case 110L   : tio.c_cflag = B110;    break;
  case 134L   : tio.c_cflag = B134;    break;
  case 150L   : tio.c_cflag = B150;    break;
  case 200L   : tio.c_cflag = B200;    break;
  case 300L   : tio.c_cflag = B300;    break;
  case 600L   : tio.c_cflag = B600;    break;
  case 1200L  : tio.c_cflag = B1200;   break;
  case 1800L  : tio.c_cflag = B1800;   break;
  case 2400L  : tio.c_cflag = B2400;   break;
  case 4800L  : tio.c_cflag = B4800;   break;
  case 9600L  : tio.c_cflag = B9600;   break;
  case 19200L : tio.c_cflag = B19200;  break;
  case 38400L : tio.c_cflag = B38400;  break;
  case 57600L : tio.c_cflag = B57600;  break;
  case 115200L: tio.c_cflag = B115200; break;
  case 230400L: tio.c_cflag = B230400; break;
  default:
    fprintf(stderr, "i2c_open_serial: baudrate %lu not supported\n", baud);
    return -5;
  }
  tio.c_cflag |= CS8 | CLOCAL | CREAD ;
  tio.c_iflag = IGNPAR;
  
  if(tcflush(fd, TCIOFLUSH) < 0){
    perror("i2c_open_serial: tcflush");
    close(fd);
    return -3;
  }
  
  if(tcsetattr(fd, TCSANOW, &tio) < 0){
    perror("i2c_open_serial: tcsetattr");
    close(fd);
    return -4;
  }

  PDEBUG("opened device %s at baudrate %lu\n", device, baud);
  
  return fd;
#endif /* WIN32 */
}



int i2c_close_serial(handle_t handle)
{
#ifdef WIN32
	if(CloseHandle(handle)){
		return 0;
	}else{
		return -1;
	}

//return(0);
#else

  if(tcdrain(handle) < 0){
    perror("i2c_close_serial: tcdrain");
    return -1;
  }

  if(tcflush(handle, TCIOFLUSH) < 0){
    perror("i2c_close_serial: tcflush");
    return -2;
  }

  if(close(handle) < 0){
    perror("i2c_close_serial: close");
    return -3;
  }
  
  return 0;
#endif
}



int i2c_write(handle_t handle,
	      uint8 mod_addr,
	      uint8 mod_reg,
	      uint8 value)
{
#ifdef WIN32

  DWORD dwNumBytesWritten;
  uint8 byte[3];
  BOOL writesuccess;
 
  byte[0] = mod_addr & 0x7F;
  byte[1] = mod_reg  & 0x7F;
  byte[2] = value;
 

  writesuccess=WriteFile (handle,				// Port handle
           byte,				// Pointer to the data to write 
           3,					// Number of bytes to write
           &dwNumBytesWritten,	// Pointer to the number of bytes 
								// written
           NULL					// Must be NULL for Windows 
		  );
	if(!writesuccess)
		return(-1);
	if(dwNumBytesWritten==3 && writesuccess)
		return 0;
	else 
		return -3;

#else
  uint8 buf[3];
  ssize_t count;
  int i;

  buf[0] = mod_addr & 0x7F;
  buf[1] = mod_reg  & 0x7F;
  buf[2] = value;
  
  PDEBUG("buffer:");
  
  for(i = 0; i < 3; ++i){
    while((count = write(handle, &(buf[i]), 1)) != 1)
      if(count < 0){
	perror("i2c_write");
	return -1;
      }
    PDEBUG_SNIP(" 0x%02X", buf[i]);
  }
  PDEBUG_SNIP("\n");
  
  if(tcdrain(handle) < 0){
    perror("i2c_write: tcdrain");
    return -2;
  }

  return 0;
#endif
}



int i2c_read(handle_t handle,
	     uint8 mod_addr,
	     uint8 mod_reg,
	     uint8 *value)
{
#ifdef WIN32


//	BYTE Byte;
	DWORD dwBytesTransferred;
	BOOL readsuccess;
	BOOL writesuccess;
	uint8 byte[2];

	
	byte[0] = mod_addr & 0x7F;
	byte[1] = mod_reg  | 0x80;

	writesuccess=WriteFile (handle,				// Port handle
           byte,				// Pointer to the data to write 
           2,					// Number of bytes to write
           &dwBytesTransferred,	// Pointer to the number of bytes 
								// written
           NULL					// Must be NULL for Windows 
		  );
	if(!writesuccess || (dwBytesTransferred!=2))
		return(-1);


	readsuccess=ReadFile (handle,  // Port handle
			value,					// Pointer to data to read
			1,						// Number of bytes to read
			&dwBytesTransferred,	// Pointer to number of bytes read
			NULL					// Must be NULL for Windows CE
 			);
	if(!readsuccess)  
		return(-3);

	if(dwBytesTransferred==1)
		return 0;
	else return(-4);

#else
  uint8 buf[2];
  ssize_t count;
  int i;

  buf[0] = mod_addr & 0x7F;
  buf[1] = mod_reg  | 0x80;
  
  PDEBUG("send buffer:");
  
  for(i = 0; i < 2; ++i){
    while((count = write(handle, &(buf[i]), 1)) != 1)
      if(count < 0){
	perror("sending request in i2c_read");
	return -1;
      }
    PDEBUG_SNIP(" 0x%02X", buf[i]);
  }
  PDEBUG_SNIP("\n");
  
  PDEBUG("receive buffer:");
  
  if(tcdrain(handle) < 0){
    perror("i2c_read: tcdrain");
    return -2;
  }
  
  while((count = read(handle, value, 1)) != 1)
    if(count < 0){
      perror("receiving answer in i2c_read");
      return -3;
    }
  PDEBUG_SNIP(" 0x%02X\n", *value);
  
  return 0;
#endif
}



int i2c_waitup(handle_t handle,
	       uint8 mod_addr,
	       uint8 mod_reg,
	       uint8 mask)
{

  int res;
  uint8 value = 0;
  
  while( ! value){
    if((res = i2c_read(handle, mod_addr, mod_reg, &value)) < 0)
      return res;
    
    value &= mask;
  }
  
  return 0;

}



int i2c_waitdown(handle_t handle,
		 uint8 mod_addr,
		 uint8 mod_reg,
		 uint8 mask)
{
  int res;
  uint8 value = 1;
  
  while(value){
    if((res = i2c_read(handle, mod_addr, mod_reg, &value)) < 0)
      return res;
    
    value &= mask;
  }
  
  return 0;
}

