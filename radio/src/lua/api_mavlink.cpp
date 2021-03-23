/*
 * Copyright (C) OpenTX
 *
 * Based on code named
 *   th9x - http://code.google.com/p/th9x
 *   er9x - http://code.google.com/p/er9x
 *   gruvin9x - http://code.google.com/p/gruvin9x
 *
 * License GPLv2: http://www.gnu.org/licenses/gpl-2.0.html
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */
/*
 *  (c) www.olliw.eu, OlliW, OlliW42
 */

#include <ctype.h>
#include <stdio.h>
#include "opentx.h"
#include "lua_api.h"
#include "thirdparty/Mavlink/opentx_lua_lib_constants.h"


static int luaMavlinkGetVersion(lua_State * L)
{
  lua_pushinteger(L, FASTMAVLINK_MAVLINK_VERSION); // this is the version reported also by the heartbeat
  return 1;
}


static int luaMavlinkGetChannelStatus(lua_State * L)
{
  lua_createtable(L, 0, 6);
  lua_pushtableinteger(L, "msg_rx_count", mavlinkTelem.msg_rx_count);
  lua_pushtableinteger(L, "msg_rx_per_sec", mavlinkTelem.msg_rx_persec);
  lua_pushtableinteger(L, "bytes_rx_per_sec", mavlinkTelem.bytes_rx_persec);
  lua_pushtableinteger(L, "msg_tx_count", mavlinkTelem.msg_tx_count);
  lua_pushtableinteger(L, "msg_tx_per_sec", mavlinkTelem.msg_tx_persec);
  lua_pushtableinteger(L, "bytes_tx_per_sec", mavlinkTelem.bytes_tx_persec);
  return 1;
}
/*
static int luaMavlinkMsgAvailable(lua_State * L)
{
  uint32_t size = mavlinkTelem.msgRxFifo.size();
  lua_pushinteger(L, size);
  return 1;
}

static int luaMavlinkMsgProbeHeader(lua_State * L)
{
mavlink_message_t msg; // FIXME: should be a reference/pointer to save stack, but _MAV_PAYLOAD wants a pointer

  if (!mavlinkTelem.msgRxFifo.probe(msg)) { lua_pushnil(L); return 1; }

  lua_createtable(L, 0, 9);
  lua_pushtableinteger(L, "len", msg.len);
  lua_pushtableinteger(L, "incompat_flags", msg.incompat_flags);
  lua_pushtableinteger(L, "compat_flags", msg.compat_flags);
  lua_pushtableinteger(L, "seq", msg.seq);
  lua_pushtableinteger(L, "sysid", msg.sysid);
  lua_pushtableinteger(L, "compid", msg.compid);
  lua_pushtableinteger(L, "msgid", msg.msgid);
  //we also like to get the target
  const mavlink_msg_entry_t* msg_entry = mavlink_get_msg_entry(msg.msgid);
  if (msg_entry && (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM)) {
  uint8_t tsystem = (uint8_t)_MAV_PAYLOAD(&msg)[msg_entry->target_system_ofs];
  lua_pushtableinteger(L, "target_system", tsystem );
  }
  else {
  lua_pushtableinteger(L, "target_system", -1 );
  }
  if (msg_entry && (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT)) {
  uint8_t tcomponent = (uint8_t)_MAV_PAYLOAD(&msg)[msg_entry->target_component_ofs];
  lua_pushtableinteger(L, "target_component", tcomponent );
  }
  else {
  lua_pushtableinteger(L, "target_component", -1 );
  }
  return 1;
}

static int luaMavlinkMsgPopAndDiscard(lua_State * L)
{
  if (!mavlinkTelem.msgRxFifo.isEmpty()) {
  mavlinkTelem.msgRxFifo.skip();
  }
  return 0;
}

static int luaMavlinkMsgClear(lua_State * L)
{
  mavlinkTelem.msgRxFifo.clear();
  return 0;
}

static int luaMavlinkMsgEnable(lua_State * L)
{
//  uint32_t flag = luaL_checkunsigned(L, 1);
//  if (flag && !mavlinkTelem.msgFifo_enabled) {
//    //we enable it from disabled state, so we better clear()
//    mavlinkTelem.msgRxFifo.clear();
//  }
//  mavlinkTelem.msgFifo_enabled = (flag > 0);
  return 0;
}
*/


#if 0

#include "api_mavlink_generated.h"

const luaL_Reg mavlinkLib[] = {
  { "getVersion", luaMavlinkGetVersion },
  { "getChannelStatus", luaMavlinkGetChannelStatus },
  { "available", luaMavlinkMsgAvailable },
  { "probeHeader", luaMavlinkMsgProbeHeader },
  { "popAndDiscard", luaMavlinkMsgPopAndDiscard },
  { "clear", luaMavlinkMsgClear },
  { "enable", luaMavlinkMsgEnable },

  MAVLINK_LIB_FUNCTIONS

  { nullptr, nullptr }  /* sentinel */
};

const luaR_value_entry mavlinkConstants[] = {
  MAVLINK_LIB_CONSTANTS
  { nullptr, 0 }  /* sentinel */
};

#else

/*
static int luaMavlinkMsgHeartbeatPop(lua_State * L)
{
mavlink_message_t msg; // FIXME: should be a reference/pointer to save stack, but _MAV_PAYLOAD wants a pointer

  if (!mavlinkTelem.msgRxFifo.pop(msg)) {
  lua_pushnil(L);
  return 1;
  }
  if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) {
  lua_pushnil(L);
  return 1;
  }

  mavlink_heartbeat_t payload;
  mavlink_msg_heartbeat_decode(&msg, &payload);

  lua_createtable(L, 0, 6);
  lua_pushtableinteger(L, "custom_mode", payload.custom_mode);
  lua_pushtableinteger(L, "type", payload.type);
  lua_pushtableinteger(L, "autopilot", payload.autopilot);
  lua_pushtableinteger(L, "base_mode", payload.base_mode);
  lua_pushtableinteger(L, "system_status", payload.system_status);
  lua_pushtableinteger(L, "mavlink_version", payload.mavlink_version);
  return 1;
}

static int luaMavlinkMsgAttitudePop(lua_State * L)
{
mavlink_message_t msg; // FIXME: should be a reference/pointer to save stack, but _MAV_PAYLOAD wants a pointer

  if (!mavlinkTelem.msgRxFifo.pop(msg)) {
  lua_pushnil(L);
  return 1;
  }
  if (msg.msgid != MAVLINK_MSG_ID_ATTITUDE) {
  lua_pushnil(L);
  return 1;
  }

  mavlink_attitude_t payload;
  mavlink_msg_attitude_decode(&msg, &payload);

  lua_createtable(L, 0, 7);
  lua_pushtableinteger(L, "time_boot_ms", payload.time_boot_ms);
  lua_pushtablenumber(L, "roll", payload.roll);
  lua_pushtablenumber(L, "pitch", payload.pitch);
  lua_pushtablenumber(L, "yaw", payload.yaw);
  lua_pushtablenumber(L, "rollspeed", payload.rollspeed);
  lua_pushtablenumber(L, "pitchspeed", payload.pitchspeed);
  lua_pushtablenumber(L, "yawspeed", payload.yawspeed);
  return 1;
}

#define MAVLINK_LIB_FUNCTIONS \
  { "msgHeartbeatPop", luaMavlinkMsgHeartbeatPop }, \
  { "msgAttitudePop", luaMavlinkMsgAttitudePop }, \
*/


//------------------------------------------------------------
// mavlink luaL and luaR arrays
//------------------------------------------------------------

const luaL_Reg mavlinkLib[] = {
  { "getVersion", luaMavlinkGetVersion },
  { "getChannelStatus", luaMavlinkGetChannelStatus },
//  { "available", luaMavlinkMsgAvailable },
//  { "probeHeader", luaMavlinkMsgProbeHeader },
//  { "popAndDiscard", luaMavlinkMsgPopAndDiscard },
//  { "clear", luaMavlinkMsgClear },
//  { "enable", luaMavlinkMsgEnable },

//  MAVLINK_LIB_FUNCTIONS

  { nullptr, nullptr }  /* sentinel */
};

const luaR_value_entry mavlinkConstants[] = {
  MAVLINK_LIB_CONSTANTS
  { nullptr, 0 }  /* sentinel */
};

#endif
