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



/** \file
 *  \brief
 *  Basic I2C communication layer.
 *
 *  <h2>Introduction</h2>
 *
 *  This layer provides basic mechanisms for communicating with ASL's
 *  I2C modules using a translator connected to a serial port. Use the
 *  function i2c_open_serial() to obtain a file descriptor of the
 *  serial port, and the functions i2c_read() and i2c_write() for
 *  low-level communication.
 *
 *
 *  <h2>How to use I2C modules and drivers</h2>
 *
 *  The I2C Hardware modules plug into the little I2C plug on the
 *  robot's I/O circuitry. I2C is a bus, that means modules can be
 *  chained, which is why each of them has two I2C connectors.
 *
 *  The I2C drivers are organized into two levels: The basic
 *  abstraction is provided by i2c.h (this file) and higher level
 *  drivers, each corresponding to one of the I2C hardware
 *  modules. The i2c_trans.h module is a little special: It
 *  corresponds to the bridge that translates RS232 to I2C, which
 *  provides some supplementary functionality and appears on the bus
 *  like any external I2C module.
 *
 *  Whereas interaction with the base driver requires a valid <em>file
 *  descriptor</em>, higher levels use <em>objects</em> to store the
 *  information needed for their operation. Thus, each high-level
 *  module has a <code>i2c_X_init()</code> function, which usually takes
 *  three parameters:
 *  <ul>
 *    <li>
 *      <code>int fd</code>: The file descriptor from i2c_open_serial().
 *    </li>
 *    <li>
 *      <code>uint8 offset</code>: The offset from the module's
 *      default address (valid for ASL modules). It is needed if you
 *      plug two or more modules of the same kind into the bus, in
 *      which case the offset depends on jumper settings of each
 *      module. You can normally use 0.
 *    </li>
 *    <li>
 *      <code>struct i2c_X_s *s</code>: A pointer to an object of the
 *      corresponding type. The object has to be allocated by the
 *      calling program, and this function is a constructor for
 *      correctly initializing the data structure.
 *    </li>
 *  </ul>
 *
 *
 *  <h3>Base Driver</h3>
 *
 *  This module is needed for two things:
 *  <ul>
 *    <li>Initialize and clean up the serial port.</li>
 *    <li>Facilitate custom protocols or extensions of high-level drivers.</li>
 *  </ul>
 *
 *  The first role being the most important: <em>You have to get a
 *  filedescriptor from i2c_open_serial() before you can initialize any
 *  high-level driver</em>.
 *
 *
 *  <h3>Module-specific Drivers</h3>
 *
 *  You'll probably use the base i2c.h layer
 *  only for opening and closing the file descriptor, and rely on one
 *  of the higher level drivers:
 *  <ul>
 *    <li> Serial to I2C converter: i2c_trans.h </li>
 *    <li> Linear camera: i2c_lincam.h </li>
 *    <li> Servo motors: i2c_servo.h </li>
 *    <li> Analog sharp IR distance sensors: i2c_ansharp.h </li>
 *    <li> Digital sharp IR distance sensors: i2c_digsharp.h </li>
 *    <li> Ultrasound distance sensor: i2c_usound.h </li>
 *  </ul>
 *
 *  There also exists some test programs (in the <code>util/</code>
 *  subdirectory) which can be used to familiarize yourself e.g. with
 *  the I2C modules. Apart from the examples provided in the
 *  subdirectory of the same name, these test programs might be good
 *  starting pointd for your programs.
 *
 */



#ifndef I2C_H
#define I2C_H


#include "utility.h"

#ifdef WIN32
# include <windows.h>
  typedef HANDLE handle_t;
#else
  typedef int handle_t;
#endif


/**
 *  Address of the modeflags register, common to all I2C modules and
 *  thus defined here.
 */
#define I2C_MODEFLAGS    0x28



/**
 *  Open the serial port to which the translator circuit is
 *  attached. Possible baud rates are
 *
 * <ul>
 *   <li> 50 </li>
 *   <li> 75 </li>
 *   <li> 110 </li>
 *   <li> 134 </li>
 *   <li> 150 </li>
 *   <li> 200 </li>
 *   <li> 300 </li>
 *   <li> 600 </li>
 *   <li> 1200 </li>
 *   <li> 1800 </li>
 *   <li> 2400 </li>
 *   <li> 4800 </li>
 *   <li> 9600 </li>
 *   <li> 19200 </li>
 *   <li> 38400 </li>
 *   <li> 57600 </li>
 *   <li> 115200 </li>
 *   <li> 230400 </li>
 * </ul>
 *
 *  \note
 *  <ul>
 *    <li> Baud rates 115200 and 230400, when given as constants in your
 *         C sourcecode, need to be specified as "115200L" and "230400L".
 *         Without the trailing "L", the compiler might not treat those
 *         large numbers correctly.</li>
 *    <li> On Smartease, device is "/dev/tts/0" (or maybe "/dev/tts/1",
 *         depending on the hardware version).</li>
 *  </ul>
 *
 *  \return
 *  <ul>
 *    <li> >=0 : openend filedescriptor </li>
 *    <li> -1  : couldn't open port </li>
 *    <li> -2  : couldn't clear termios structure </li>
 *    <li> -3  : couldn't flush port </li>
 *    <li> -4  : couldn't set port attributes </li>
 *    <li> -5  : invalid baud rate </li>
 *  </ul>
 * */
handle_t i2c_open_serial(const char *device, //!< device file filesystem emtry
			 unsigned long baud, //!< wanted baud rate
			 unsigned long timeoutmiliseconde //!< wanted time out in ms
			 );



/**
 *  Close the serial port (after flushing).
 * 
 *  \note
 *  <ul>
 *    <li> If error -1 occurs, the file descriptor is not closed. </li>
 *  </ul>
 * 
 *  \return
 *  <ul>
 *    <li> 0 : success </li>
 *    <li> -1 : couldn't drain port </li>
 *    <li> -2 : couldn't flush port </li>
 *    <li> -3 : couldn't close port </li>
 *  </ul>
 * 
 */
int i2c_close_serial(handle_t handle //!< filedescriptor of serial port
		     );



/**
 *  Write to a register of an I2C module.
 *
 *  \note
 *  <ul>
 *    <li> Only the 7 least significant bits of mod_addr and mod_reg are used.
 *    </li>
 *  </ul>
 *
 * \todo Check that timeout is really implemented in both Windows and POSIX.
 * 
 *  \return
 *  <ul>
 *    <li> 0 : success </li>
 *    <li> -1 : write error </li>
 *    <li> -2 : couldn't drain port </li>
 *	  <li> -3 : write timeout </li>
 *  </ul>
 * 
 */
int i2c_write(handle_t handle,	//!< file descriptor
	      uint8 mod_addr,	//!< i2c address of module
	      uint8 mod_reg,	//!< address of register inside module
	      uint8 value	//!< value to be written
	      );


/**
 *  Read a register of an I2C module.
 * 
 *  \note
 *  <ul>
 *    <li> Only the 7 least significant bits of mod_addr and mod_reg are used.
 *    </li>
 *  </ul>
 * 
 *  \return
 *  <ul>
 *    <li> 0 : success </li>
 *    <li> -1 : error sending the read request </li>
 *    <li> -2 : couldn't drain port </li>
 *    <li> -3 : error receiving the answer </li>
 *	  <li> -4 : read timeout </li>
 *  </ul>
 * 
 */
int i2c_read(handle_t handle,	//!< file descriptor
	     uint8 mod_addr,	//!< i2c address of module
	     uint8 mod_reg,	//!< address of register inside module
	     uint8 *value       //!< the value of the register (if success)
	     );



/**
 *  Wait (indefinetely) for (any) bit of a register to
 *  become 1 without sleeping (busy wait).
 *
 * \todo Add a timeout.
 * 
 *  \return
 *  <ul>
 *    <li> 0 : success </li>
 *    <li> <0 : same as i2c_read() </li>
 *  </ul>
 * 
 */
int i2c_waitup(handle_t handle,	//!< file descriptor
	       uint8 mod_addr,	//!< i2c address of module
	       uint8 mod_reg,	//!< address of register inside module
	       uint8 mask       //!< defines the bits to wait for
	       );



/**
 *  Wait (indefinetely) for (any) bit of a register to
 *  become 0 without sleeping (busy wait).
 * 
 *  \return
 *  <ul>
 *    <li> 0 : success </li>
 *    <li> <0 : same as i2c_read() </li>
 *  </ul>
 * 
 */
int i2c_waitdown(handle_t handle, //!< file descriptor
		 uint8 mod_addr,  //!< i2c address of module
		 uint8 mod_reg,	  //!< address of register inside module
		 uint8 mask	  //!< defines the bits to wait for
		 );

#endif // I2C_H
