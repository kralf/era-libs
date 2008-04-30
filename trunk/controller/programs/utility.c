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



#include "utility.h"

#include <stdio.h>

#ifdef WIN32
#else
#include <unistd.h>
#endif /*WIN32*/



/** \file
 *  \brief
 *  Useful and common stuff implementation.
 *
 *  Compiles into a relocatable object file that then becomes part of
 *  the libsmartease.so library.
 */



double usecdiff(struct timeval *p,
		struct timeval *m)
{
  return 1e6  * (p->tv_sec - m->tv_sec) + p->tv_usec - m->tv_usec;
}



int busysleep(double usec)
{
  // it would probably be a good idea to use the timercmp macro,
  // described in the gettimeofday manpage
  struct timeval t0, t1;

  if(gettimeofday(&t0, NULL) < 0)
    return -1;

  do{
    if(gettimeofday(&t1, NULL) < 0)
      return -1;
  } while(usecdiff(&t1, &t0) < usec);

  return 0;
}



int writebuffer(int fd,
		const void *buffer,
		int n_bytes)
{
  int n;
  
  while(n_bytes > 0){
    while((n = write(fd, buffer, n_bytes)) == 0);
    if(n < 0){
      perror("writebuffer");
      return -1;
    }
    
    n_bytes -= n;
    buffer  += n;
  }

  return 0;
}



int readbuffer(int fd,
	       void *buffer,
	       int n_bytes)
{
  int n;
  
  while(n_bytes > 0){
    while((n = read(fd, buffer, n_bytes)) == 0);
    if(n < 0){
      perror("readbuffer");
      return -1;
    }
    
    n_bytes -= n;
    buffer  += n;
  }
  
  return 0;
}
