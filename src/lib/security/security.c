/***************************************************************************
 *   Copyright (C) 2008 by Ralf Kaestner                                   *
 *   ralf.kaestner@gmail.com                                               *
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

#include "motors/home.h"

#include "security/security.h"

const char* era_security_errors[] = {
  "success",
  "error enabling security",
  "error disabling security",
};

void era_security_init(era_security_p security, era_security_func_t func, 
  int estop_channel, int switch_channel) {
  security->func = func;

  security->estop_channel = estop_channel;
  security->switch_channel = switch_channel;
}

int era_security_enable(era_security_p security, era_motors_p motors) {
  int i, result = EPOS_INPUT_ERROR_NONE;
  epos_node_p node_a = (epos_node_p)motors;

  epos_input_func_t estop_func = 
    {security->estop_channel, epos_input_low, 1, 1};
  epos_input_func_t switch_func = 
    {security->switch_channel, epos_input_high, 1, 1};

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i) {
    result |= epos_input_set_func(&node_a[i].input, 
      (security->func == era_security_neg_switch_pos_estop) ?
      epos_input_pos_switch : epos_input_neg_switch, &estop_func);
    result |= epos_input_set_func(&node_a[i].input, 
      (security->func == era_security_neg_switch_pos_estop) ?
      epos_input_neg_switch : epos_input_pos_switch, &switch_func);
  }

  if (result)
    return ERA_SECURITY_ERROR_ENABLE;
  else
    return ERA_SECURITY_ERROR_NONE;
}

int era_security_disable(era_security_p security, era_motors_p motors) {
  int i, result = EPOS_INPUT_ERROR_NONE;
  epos_node_p node_a = (epos_node_p)motors;

  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i) {
    result |= epos_input_set_enabled(&node_a[i].input, 
      epos_input_neg_switch, 0);
    result |= epos_input_set_enabled(&node_a[i].input, 
      epos_input_pos_switch, 0);
  }

  if (result)
    return ERA_SECURITY_ERROR_DISABLE;
  else
    return ERA_SECURITY_ERROR_NONE;
}

int era_security_enable_home(era_security_p security, era_motors_p motors) {
  int i, result = EPOS_INPUT_ERROR_NONE;
  epos_node_p node_a = (epos_node_p)motors;

  /* Check for limit switch states */
  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i) {
    if (epos_input_get_func_state(&node_a[i].input, epos_input_neg_switch) ||
      epos_input_get_func_state(&node_a[i].input, epos_input_pos_switch))
      return ERA_SECURITY_ERROR_ENABLE;
  }

  /* Disable limit switch functionality */
  epos_input_func_t estop_func = 
    {security->estop_channel, epos_input_low, 1, 1};
  epos_input_func_t switch_func = 
    {security->switch_channel, epos_input_high, 0, 1};
  for (i = 0; i < sizeof(era_motors_t)/sizeof(epos_node_t); ++i) {
    epos_home_method_t method = config_get_int(&node_a[i].config, 
      EPOS_PARAMETER_HOME_METHOD);

    era_security_func_t func = ((method == epos_home_neg_switch) ||
      (method == epos_home_neg_switch_index)) ? 
      era_security_neg_switch_pos_estop : era_security_pos_switch_neg_estop;

    result |= epos_input_set_func(&node_a[i].input, 
      (func == era_security_neg_switch_pos_estop) ?
      epos_input_pos_switch : epos_input_neg_switch, &estop_func);
    result |= epos_input_set_func(&node_a[i].input, 
      (func == era_security_neg_switch_pos_estop) ?
      epos_input_neg_switch : epos_input_pos_switch, &switch_func);
  }

  if (result)
    return ERA_SECURITY_ERROR_ENABLE;
  else
    return ERA_SECURITY_ERROR_NONE;
}
