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



#ifndef UTILITY_H
#define UTILITY_H



/** \file
 *  \brief
 *  Useful and common stuff.
 *
 *  Provide some useful functionality used inside the drivers and
 *  applications for Smartease.
 */


#ifdef WIN32
#else
#include <sys/time.h>
#endif /*WIN32*/


#ifndef NULL
#define NULL 0
#endif



/**
 * Returns the absolute value of x.
 *
 * \note Due to macro expansion, the expression x is evaluated twice.
 */
#define ABSVAL(x)    ((x) >=   0 ? (x) : -(x))



/**
 * Returns the smaller of two numbers x and y.
 *
 * \note Due to macro expansion, the expressions x and y are evaluated twice.
 */
#define MINVAL(x, y) ((x) <= (y) ? (x) :  (y))



/**
 * Returns the bigger of two numbers x and y.
 *
 * \note Due to macro expansion, the expressions x and y are evaluated twice.
 */
#define MAXVAL(x, y) ((x) >= (y) ? (x) :  (y))



/**
 * Unsigned 8-bit value.
 *
 * \note Platform specific (works on Smartease).
 */
typedef unsigned char      uint8;



/**
 * Unsigned 16-bit value.
 *
 * \note Platform specific (works on Smartease).
 */
typedef unsigned short int uint16;



/**
 * Unsigned 32-bit value.
 *
 * \note Platform specific (works on Smartease).
 */
typedef unsigned int       uint32;



/**
 * Signed 8-bit value.
 *
 * \note Platform specific (works on Smartease).
 */
typedef          char      int8;



/**
 * Signed 16-bit value.
 *
 * \note Platform specific (works on Smartease).
 */
typedef          short int int16;



/**
 * Signed 32-bit value.
 *
 * \note Platform specific (works on Smartease).
 */
typedef          int       int32;



#ifndef WIN32
/**
 * Returns the difference in microseconds between two time values.
 */
double usecdiff(struct timeval *p, //!< timevalue counted positively
		struct timeval *m  //!< timevalue counted negaitively
		);



/**
 * Implements sleeping by looping until usec microseconds have
 * passed. Useful for short waits when functions such as usleep would
 * put the process to sleep to schedule another one.
 *
 * \return 0 on success, -1 if the sleep was aborted due to an error
 * in gettimeofday() (in which case the error is described in the C
 * library's errno, a description of which can be printed using the
 * perror() library call).
 */
int busysleep(double usec	//!< number of microseconds to sleep
	       );


/**
 * Writes the contents of a buffer to a filedescriptor, doesn't return
 * until all bytes have been written (or an error occured).
 *
 * \todo Write a similar function that includes a timeout parameter
 * and returns the number of bytes actually written, or maybe the
 * remaining number of bytes.
 *
 * \return 0 on success, -1 otherwise
 */
int writebuffer(int fd,		    //!< filedescriptor to write to
		const void *buffer, //!< pointer to the data
		int n_bytes         //!< number of bytes to write
		);



/**
 * Just like writebuffer(), only the other way around.
 */
int readbuffer(int fd,             //!< filedescriptor to write to
	       void *buffer, //!< pointer to the data
	       int n_bytes         //!< number of bytes to read
	       );

#endif

#endif // UTILITY_H
