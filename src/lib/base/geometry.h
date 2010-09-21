/***************************************************************************
 *   Copyright (C) 2008 by Fritz Stoeckli, Ralf Kaestner                   *
 *   stfritz@ethz.ch, ralf.kaestner@gmail.com                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef ERA_GEOMETRY_H
#define ERA_GEOMETRY_H

/** \brief ERA geometric model
  * Geometric model of the BlueBotics ERA-5/1 robot arm.
  */

/** \brief Structure defining the arm geometry
  */
typedef struct era_geometry_t {
  double upper_length;  //!< The upper arm's length in [m].
  double lower_length;  //!< The forearm's length in [m].

  double tool_length;   //!< The tool length in [m].
} era_geometry_t, *era_geometry_p;

/** \brief Initialize the geometric arm model
  * \param[in] geometry The geometric arm model to be initialized.
  * \param[in] upper_length The upper arm's length in [m].
  * \param[in] lower_length The lower arm's length in [m].
  * \param[in] tool_length The tool length in [m].
  */
void era_geometry_init(
  era_geometry_p geometry,
  double upper_length,
  double lower_length,
  double tool_length);

#endif
