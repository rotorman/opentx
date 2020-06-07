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
#include "thirdparty/Mavlink/c_library_v2/common/mavlink.h"


static int luaMavlinkGetVersion(lua_State * L)
{
    lua_pushinteger(L, 3); // this is the version reported also by the heartbeat, unfortunately there is no define so we need to do by hand
    lua_pushstring(L, MAVLINK_WIRE_PROTOCOL_VERSION);
    return 2;
}


static int luaMavlinkGetChannelStatus(lua_State * L)
{
	//mavlink_status_t* status = mavlink_get_channel_status(MAVLINK_COMM_0);
	const mavlink_status_t* status = mavlinkTelem.getChannelStatus();

    lua_createtable(L, 0, 13);
    lua_pushtableinteger(L, "msg_received", status->msg_received);
 	lua_pushtableinteger(L, "buffer_overrun", status->buffer_overrun);
    lua_pushtableinteger(L, "parse_error", status->parse_error);
    lua_pushtableinteger(L, "parse_state", (uint8_t)status->parse_state);
    lua_pushtableinteger(L, "packet_idx", status->packet_idx);
    lua_pushtableinteger(L, "current_rx_seq", status->current_rx_seq);
    lua_pushtableinteger(L, "current_tx_seq", status->current_tx_seq);
    lua_pushtableinteger(L, "packet_rx_success_count", status->packet_rx_success_count);
    lua_pushtableinteger(L, "packet_rx_drop_count", status->packet_rx_drop_count);

    lua_pushtableinteger(L, "msg_rx_count", mavlinkTelem.msg_rx_count);
    lua_pushtableinteger(L, "msg_rx_per_sec", mavlinkTelem.msg_rx_persec);
    lua_pushtableinteger(L, "bytes_rx_per_sec", mavlinkTelem.bytes_rx_persec);
    lua_pushtableinteger(L, "msg_rx_lost", mavlinkTelem.msg_rx_lost);
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
	} else {
		lua_pushtableinteger(L, "target_system", -1 );
	}
    if (msg_entry && (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT)) {
    	uint8_t tcomponent = (uint8_t)_MAV_PAYLOAD(&msg)[msg_entry->target_component_ofs];
        lua_pushtableinteger(L, "target_component", tcomponent );
    } else {
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
//	uint32_t flag = luaL_checkunsigned(L, 1);
//	if (flag && !mavlinkTelem.msgFifo_enabled) {
//		//we enable it from disabled state, so we better clear()
//		mavlinkTelem.msgRxFifo.clear();
//	}
//	mavlinkTelem.msgFifo_enabled = (flag > 0);
    return 0;
}
*/


#if 0

#include "api_mavlink_generated.h"

const luaL_Reg mavlinkLib[] = {
#if defined(MAVLINK_TELEM)
    { "getVersion", luaMavlinkGetVersion },
    { "getChannelStatus", luaMavlinkGetChannelStatus },
    { "available", luaMavlinkMsgAvailable },
    { "probeHeader", luaMavlinkMsgProbeHeader },
    { "popAndDiscard", luaMavlinkMsgPopAndDiscard },
    { "clear", luaMavlinkMsgClear },
    { "enable", luaMavlinkMsgEnable },

    MAVLINK_LIB_FUNCTIONS
#endif
    { nullptr, nullptr }  /* sentinel */
};

const luaR_value_entry mavlinkConstants[] = {
#if defined(MAVLINK_TELEM)
    MAVLINK_LIB_CONSTANTS
#endif
  { nullptr, 0 }  /* sentinel */
};

#else

/*
static int luaMavlinkMsgHeartbeatPop(lua_State * L)
{
mavlink_message_t msg; // FIXME: should be a reference/pointer to save stack, but _MAV_PAYLOAD wants a pointer

    if (!mavlinkTelem.msgRxFifo.pop(msg)) { lua_pushnil(L); return 1; }
    if (msg.msgid != MAVLINK_MSG_ID_HEARTBEAT) { lua_pushnil(L); return 1; }

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

    if (!mavlinkTelem.msgRxFifo.pop(msg)) { lua_pushnil(L); return 1; }
    if (msg.msgid != MAVLINK_MSG_ID_ATTITUDE) { lua_pushnil(L); return 1; }

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
// mavlinkConstants
//------------------------------------------------------------
// all the constants, but from common.xml only

#define MAVLINK_LIB_CONSTANTS \
    { "MSG_ID_HEARTBEAT", MAVLINK_MSG_ID_HEARTBEAT }, \
    { "MSG_ID_SYS_STATUS", MAVLINK_MSG_ID_SYS_STATUS }, \
    { "MSG_ID_SYSTEM_TIME", MAVLINK_MSG_ID_SYSTEM_TIME }, \
    { "MSG_ID_PING", MAVLINK_MSG_ID_PING }, \
    { "MSG_ID_CHANGE_OPERATOR_CONTROL", MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL }, \
    { "MSG_ID_CHANGE_OPERATOR_CONTROL_ACK", MAVLINK_MSG_ID_CHANGE_OPERATOR_CONTROL_ACK }, \
    { "MSG_ID_AUTH_KEY", MAVLINK_MSG_ID_AUTH_KEY }, \
    { "MSG_ID_LINK_NODE_STATUS", MAVLINK_MSG_ID_LINK_NODE_STATUS }, \
    { "MSG_ID_SET_MODE", MAVLINK_MSG_ID_SET_MODE }, \
    { "MSG_ID_PARAM_REQUEST_READ", MAVLINK_MSG_ID_PARAM_REQUEST_READ }, \
    { "MSG_ID_PARAM_REQUEST_LIST", MAVLINK_MSG_ID_PARAM_REQUEST_LIST }, \
    { "MSG_ID_PARAM_VALUE", MAVLINK_MSG_ID_PARAM_VALUE }, \
    { "MSG_ID_PARAM_SET", MAVLINK_MSG_ID_PARAM_SET }, \
    { "MSG_ID_GPS_RAW_INT", MAVLINK_MSG_ID_GPS_RAW_INT }, \
    { "MSG_ID_GPS_STATUS", MAVLINK_MSG_ID_GPS_STATUS }, \
    { "MSG_ID_SCALED_IMU", MAVLINK_MSG_ID_SCALED_IMU }, \
    { "MSG_ID_RAW_IMU", MAVLINK_MSG_ID_RAW_IMU }, \
    { "MSG_ID_RAW_PRESSURE", MAVLINK_MSG_ID_RAW_PRESSURE }, \
    { "MSG_ID_SCALED_PRESSURE", MAVLINK_MSG_ID_SCALED_PRESSURE }, \
    { "MSG_ID_ATTITUDE", MAVLINK_MSG_ID_ATTITUDE }, \
    { "MSG_ID_ATTITUDE_QUATERNION", MAVLINK_MSG_ID_ATTITUDE_QUATERNION }, \
    { "MSG_ID_LOCAL_POSITION_NED", MAVLINK_MSG_ID_LOCAL_POSITION_NED }, \
    { "MSG_ID_GLOBAL_POSITION_INT", MAVLINK_MSG_ID_GLOBAL_POSITION_INT }, \
    { "MSG_ID_RC_CHANNELS_SCALED", MAVLINK_MSG_ID_RC_CHANNELS_SCALED }, \
    { "MSG_ID_RC_CHANNELS_RAW", MAVLINK_MSG_ID_RC_CHANNELS_RAW }, \
    { "MSG_ID_SERVO_OUTPUT_RAW", MAVLINK_MSG_ID_SERVO_OUTPUT_RAW }, \
    { "MSG_ID_MISSION_REQUEST_PARTIAL_LIST", MAVLINK_MSG_ID_MISSION_REQUEST_PARTIAL_LIST }, \
    { "MSG_ID_MISSION_WRITE_PARTIAL_LIST", MAVLINK_MSG_ID_MISSION_WRITE_PARTIAL_LIST }, \
    { "MSG_ID_MISSION_ITEM", MAVLINK_MSG_ID_MISSION_ITEM }, \
    { "MSG_ID_MISSION_REQUEST", MAVLINK_MSG_ID_MISSION_REQUEST }, \
    { "MSG_ID_MISSION_SET_CURRENT", MAVLINK_MSG_ID_MISSION_SET_CURRENT }, \
    { "MSG_ID_MISSION_CURRENT", MAVLINK_MSG_ID_MISSION_CURRENT }, \
    { "MSG_ID_MISSION_REQUEST_LIST", MAVLINK_MSG_ID_MISSION_REQUEST_LIST }, \
    { "MSG_ID_MISSION_COUNT", MAVLINK_MSG_ID_MISSION_COUNT }, \
    { "MSG_ID_MISSION_CLEAR_ALL", MAVLINK_MSG_ID_MISSION_CLEAR_ALL }, \
    { "MSG_ID_MISSION_ITEM_REACHED", MAVLINK_MSG_ID_MISSION_ITEM_REACHED }, \
    { "MSG_ID_MISSION_ACK", MAVLINK_MSG_ID_MISSION_ACK }, \
    { "MSG_ID_SET_GPS_GLOBAL_ORIGIN", MAVLINK_MSG_ID_SET_GPS_GLOBAL_ORIGIN }, \
    { "MSG_ID_GPS_GLOBAL_ORIGIN", MAVLINK_MSG_ID_GPS_GLOBAL_ORIGIN }, \
    { "MSG_ID_PARAM_MAP_RC", MAVLINK_MSG_ID_PARAM_MAP_RC }, \
    { "MSG_ID_MISSION_REQUEST_INT", MAVLINK_MSG_ID_MISSION_REQUEST_INT }, \
    { "MSG_ID_MISSION_CHANGED", MAVLINK_MSG_ID_MISSION_CHANGED }, \
    { "MSG_ID_SAFETY_SET_ALLOWED_AREA", MAVLINK_MSG_ID_SAFETY_SET_ALLOWED_AREA }, \
    { "MSG_ID_SAFETY_ALLOWED_AREA", MAVLINK_MSG_ID_SAFETY_ALLOWED_AREA }, \
    { "MSG_ID_ATTITUDE_QUATERNION_COV", MAVLINK_MSG_ID_ATTITUDE_QUATERNION_COV }, \
    { "MSG_ID_NAV_CONTROLLER_OUTPUT", MAVLINK_MSG_ID_NAV_CONTROLLER_OUTPUT }, \
    { "MSG_ID_GLOBAL_POSITION_INT_COV", MAVLINK_MSG_ID_GLOBAL_POSITION_INT_COV }, \
    { "MSG_ID_LOCAL_POSITION_NED_COV", MAVLINK_MSG_ID_LOCAL_POSITION_NED_COV }, \
    { "MSG_ID_RC_CHANNELS", MAVLINK_MSG_ID_RC_CHANNELS }, \
    { "MSG_ID_REQUEST_DATA_STREAM", MAVLINK_MSG_ID_REQUEST_DATA_STREAM }, \
    { "MSG_ID_DATA_STREAM", MAVLINK_MSG_ID_DATA_STREAM }, \
    { "MSG_ID_MANUAL_CONTROL", MAVLINK_MSG_ID_MANUAL_CONTROL }, \
    { "MSG_ID_RC_CHANNELS_OVERRIDE", MAVLINK_MSG_ID_RC_CHANNELS_OVERRIDE }, \
    { "MSG_ID_MISSION_ITEM_INT", MAVLINK_MSG_ID_MISSION_ITEM_INT }, \
    { "MSG_ID_VFR_HUD", MAVLINK_MSG_ID_VFR_HUD }, \
    { "MSG_ID_COMMAND_INT", MAVLINK_MSG_ID_COMMAND_INT }, \
    { "MSG_ID_COMMAND_LONG", MAVLINK_MSG_ID_COMMAND_LONG }, \
    { "MSG_ID_COMMAND_ACK", MAVLINK_MSG_ID_COMMAND_ACK }, \
    { "MSG_ID_MANUAL_SETPOINT", MAVLINK_MSG_ID_MANUAL_SETPOINT }, \
    { "MSG_ID_SET_ATTITUDE_TARGET", MAVLINK_MSG_ID_SET_ATTITUDE_TARGET }, \
    { "MSG_ID_ATTITUDE_TARGET", MAVLINK_MSG_ID_ATTITUDE_TARGET }, \
    { "MSG_ID_SET_POSITION_TARGET_LOCAL_NED", MAVLINK_MSG_ID_SET_POSITION_TARGET_LOCAL_NED }, \
    { "MSG_ID_POSITION_TARGET_LOCAL_NED", MAVLINK_MSG_ID_POSITION_TARGET_LOCAL_NED }, \
    { "MSG_ID_SET_POSITION_TARGET_GLOBAL_INT", MAVLINK_MSG_ID_SET_POSITION_TARGET_GLOBAL_INT }, \
    { "MSG_ID_POSITION_TARGET_GLOBAL_INT", MAVLINK_MSG_ID_POSITION_TARGET_GLOBAL_INT }, \
    { "MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET", MAVLINK_MSG_ID_LOCAL_POSITION_NED_SYSTEM_GLOBAL_OFFSET }, \
    { "MSG_ID_HIL_STATE", MAVLINK_MSG_ID_HIL_STATE }, \
    { "MSG_ID_HIL_CONTROLS", MAVLINK_MSG_ID_HIL_CONTROLS }, \
    { "MSG_ID_HIL_RC_INPUTS_RAW", MAVLINK_MSG_ID_HIL_RC_INPUTS_RAW }, \
    { "MSG_ID_HIL_ACTUATOR_CONTROLS", MAVLINK_MSG_ID_HIL_ACTUATOR_CONTROLS }, \
    { "MSG_ID_OPTICAL_FLOW", MAVLINK_MSG_ID_OPTICAL_FLOW }, \
    { "MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE", MAVLINK_MSG_ID_GLOBAL_VISION_POSITION_ESTIMATE }, \
    { "MSG_ID_VISION_POSITION_ESTIMATE", MAVLINK_MSG_ID_VISION_POSITION_ESTIMATE }, \
    { "MSG_ID_VISION_SPEED_ESTIMATE", MAVLINK_MSG_ID_VISION_SPEED_ESTIMATE }, \
    { "MSG_ID_VICON_POSITION_ESTIMATE", MAVLINK_MSG_ID_VICON_POSITION_ESTIMATE }, \
    { "MSG_ID_HIGHRES_IMU", MAVLINK_MSG_ID_HIGHRES_IMU }, \
    { "MSG_ID_OPTICAL_FLOW_RAD", MAVLINK_MSG_ID_OPTICAL_FLOW_RAD }, \
    { "MSG_ID_HIL_SENSOR", MAVLINK_MSG_ID_HIL_SENSOR }, \
    { "MSG_ID_SIM_STATE", MAVLINK_MSG_ID_SIM_STATE }, \
    { "MSG_ID_RADIO_STATUS", MAVLINK_MSG_ID_RADIO_STATUS }, \
    { "MSG_ID_FILE_TRANSFER_PROTOCOL", MAVLINK_MSG_ID_FILE_TRANSFER_PROTOCOL }, \
    { "MSG_ID_TIMESYNC", MAVLINK_MSG_ID_TIMESYNC }, \
    { "MSG_ID_CAMERA_TRIGGER", MAVLINK_MSG_ID_CAMERA_TRIGGER }, \
    { "MSG_ID_HIL_GPS", MAVLINK_MSG_ID_HIL_GPS }, \
    { "MSG_ID_HIL_OPTICAL_FLOW", MAVLINK_MSG_ID_HIL_OPTICAL_FLOW }, \
    { "MSG_ID_HIL_STATE_QUATERNION", MAVLINK_MSG_ID_HIL_STATE_QUATERNION }, \
    { "MSG_ID_SCALED_IMU2", MAVLINK_MSG_ID_SCALED_IMU2 }, \
    { "MSG_ID_LOG_REQUEST_LIST", MAVLINK_MSG_ID_LOG_REQUEST_LIST }, \
    { "MSG_ID_LOG_ENTRY", MAVLINK_MSG_ID_LOG_ENTRY }, \
    { "MSG_ID_LOG_REQUEST_DATA", MAVLINK_MSG_ID_LOG_REQUEST_DATA }, \
    { "MSG_ID_LOG_DATA", MAVLINK_MSG_ID_LOG_DATA }, \
    { "MSG_ID_LOG_ERASE", MAVLINK_MSG_ID_LOG_ERASE }, \
    { "MSG_ID_LOG_REQUEST_END", MAVLINK_MSG_ID_LOG_REQUEST_END }, \
    { "MSG_ID_GPS_INJECT_DATA", MAVLINK_MSG_ID_GPS_INJECT_DATA }, \
    { "MSG_ID_GPS2_RAW", MAVLINK_MSG_ID_GPS2_RAW }, \
    { "MSG_ID_POWER_STATUS", MAVLINK_MSG_ID_POWER_STATUS }, \
    { "MSG_ID_SERIAL_CONTROL", MAVLINK_MSG_ID_SERIAL_CONTROL }, \
    { "MSG_ID_GPS_RTK", MAVLINK_MSG_ID_GPS_RTK }, \
    { "MSG_ID_GPS2_RTK", MAVLINK_MSG_ID_GPS2_RTK }, \
    { "MSG_ID_SCALED_IMU3", MAVLINK_MSG_ID_SCALED_IMU3 }, \
    { "MSG_ID_DATA_TRANSMISSION_HANDSHAKE", MAVLINK_MSG_ID_DATA_TRANSMISSION_HANDSHAKE }, \
    { "MSG_ID_ENCAPSULATED_DATA", MAVLINK_MSG_ID_ENCAPSULATED_DATA }, \
    { "MSG_ID_DISTANCE_SENSOR", MAVLINK_MSG_ID_DISTANCE_SENSOR }, \
    { "MSG_ID_TERRAIN_REQUEST", MAVLINK_MSG_ID_TERRAIN_REQUEST }, \
    { "MSG_ID_TERRAIN_DATA", MAVLINK_MSG_ID_TERRAIN_DATA }, \
    { "MSG_ID_TERRAIN_CHECK", MAVLINK_MSG_ID_TERRAIN_CHECK }, \
    { "MSG_ID_TERRAIN_REPORT", MAVLINK_MSG_ID_TERRAIN_REPORT }, \
    { "MSG_ID_SCALED_PRESSURE2", MAVLINK_MSG_ID_SCALED_PRESSURE2 }, \
    { "MSG_ID_ATT_POS_MOCAP", MAVLINK_MSG_ID_ATT_POS_MOCAP }, \
    { "MSG_ID_SET_ACTUATOR_CONTROL_TARGET", MAVLINK_MSG_ID_SET_ACTUATOR_CONTROL_TARGET }, \
    { "MSG_ID_ACTUATOR_CONTROL_TARGET", MAVLINK_MSG_ID_ACTUATOR_CONTROL_TARGET }, \
    { "MSG_ID_ALTITUDE", MAVLINK_MSG_ID_ALTITUDE }, \
    { "MSG_ID_RESOURCE_REQUEST", MAVLINK_MSG_ID_RESOURCE_REQUEST }, \
    { "MSG_ID_SCALED_PRESSURE3", MAVLINK_MSG_ID_SCALED_PRESSURE3 }, \
    { "MSG_ID_FOLLOW_TARGET", MAVLINK_MSG_ID_FOLLOW_TARGET }, \
    { "MSG_ID_CONTROL_SYSTEM_STATE", MAVLINK_MSG_ID_CONTROL_SYSTEM_STATE }, \
    { "MSG_ID_BATTERY_STATUS", MAVLINK_MSG_ID_BATTERY_STATUS }, \
    { "MSG_ID_AUTOPILOT_VERSION", MAVLINK_MSG_ID_AUTOPILOT_VERSION }, \
    { "MSG_ID_LANDING_TARGET", MAVLINK_MSG_ID_LANDING_TARGET }, \
    { "MSG_ID_FENCE_STATUS", MAVLINK_MSG_ID_FENCE_STATUS }, \
    { "MSG_ID_ESTIMATOR_STATUS", MAVLINK_MSG_ID_ESTIMATOR_STATUS }, \
    { "MSG_ID_WIND_COV", MAVLINK_MSG_ID_WIND_COV }, \
    { "MSG_ID_GPS_INPUT", MAVLINK_MSG_ID_GPS_INPUT }, \
    { "MSG_ID_GPS_RTCM_DATA", MAVLINK_MSG_ID_GPS_RTCM_DATA }, \
    { "MSG_ID_HIGH_LATENCY", MAVLINK_MSG_ID_HIGH_LATENCY }, \
    { "MSG_ID_HIGH_LATENCY2", MAVLINK_MSG_ID_HIGH_LATENCY2 }, \
    { "MSG_ID_VIBRATION", MAVLINK_MSG_ID_VIBRATION }, \
    { "MSG_ID_HOME_POSITION", MAVLINK_MSG_ID_HOME_POSITION }, \
    { "MSG_ID_SET_HOME_POSITION", MAVLINK_MSG_ID_SET_HOME_POSITION }, \
    { "MSG_ID_MESSAGE_INTERVAL", MAVLINK_MSG_ID_MESSAGE_INTERVAL }, \
    { "MSG_ID_EXTENDED_SYS_STATE", MAVLINK_MSG_ID_EXTENDED_SYS_STATE }, \
    { "MSG_ID_ADSB_VEHICLE", MAVLINK_MSG_ID_ADSB_VEHICLE }, \
    { "MSG_ID_COLLISION", MAVLINK_MSG_ID_COLLISION }, \
    { "MSG_ID_V2_EXTENSION", MAVLINK_MSG_ID_V2_EXTENSION }, \
    { "MSG_ID_MEMORY_VECT", MAVLINK_MSG_ID_MEMORY_VECT }, \
    { "MSG_ID_DEBUG_VECT", MAVLINK_MSG_ID_DEBUG_VECT }, \
    { "MSG_ID_NAMED_VALUE_FLOAT", MAVLINK_MSG_ID_NAMED_VALUE_FLOAT }, \
    { "MSG_ID_NAMED_VALUE_INT", MAVLINK_MSG_ID_NAMED_VALUE_INT }, \
    { "MSG_ID_STATUSTEXT", MAVLINK_MSG_ID_STATUSTEXT }, \
    { "MSG_ID_DEBUG", MAVLINK_MSG_ID_DEBUG }, \
    { "MSG_ID_SETUP_SIGNING", MAVLINK_MSG_ID_SETUP_SIGNING }, \
    { "MSG_ID_BUTTON_CHANGE", MAVLINK_MSG_ID_BUTTON_CHANGE }, \
    { "MSG_ID_PLAY_TUNE", MAVLINK_MSG_ID_PLAY_TUNE }, \
    { "MSG_ID_CAMERA_INFORMATION", MAVLINK_MSG_ID_CAMERA_INFORMATION }, \
    { "MSG_ID_CAMERA_SETTINGS", MAVLINK_MSG_ID_CAMERA_SETTINGS }, \
    { "MSG_ID_STORAGE_INFORMATION", MAVLINK_MSG_ID_STORAGE_INFORMATION }, \
    { "MSG_ID_CAMERA_CAPTURE_STATUS", MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS }, \
    { "MSG_ID_CAMERA_IMAGE_CAPTURED", MAVLINK_MSG_ID_CAMERA_IMAGE_CAPTURED }, \
    { "MSG_ID_FLIGHT_INFORMATION", MAVLINK_MSG_ID_FLIGHT_INFORMATION }, \
    { "MSG_ID_MOUNT_ORIENTATION", MAVLINK_MSG_ID_MOUNT_ORIENTATION }, \
    { "MSG_ID_LOGGING_DATA", MAVLINK_MSG_ID_LOGGING_DATA }, \
    { "MSG_ID_LOGGING_DATA_ACKED", MAVLINK_MSG_ID_LOGGING_DATA_ACKED }, \
    { "MSG_ID_LOGGING_ACK", MAVLINK_MSG_ID_LOGGING_ACK }, \
    { "MSG_ID_VIDEO_STREAM_INFORMATION", MAVLINK_MSG_ID_VIDEO_STREAM_INFORMATION }, \
    { "MSG_ID_VIDEO_STREAM_STATUS", MAVLINK_MSG_ID_VIDEO_STREAM_STATUS }, \
    { "MSG_ID_WIFI_CONFIG_AP", MAVLINK_MSG_ID_WIFI_CONFIG_AP }, \
    { "MSG_ID_PROTOCOL_VERSION", MAVLINK_MSG_ID_PROTOCOL_VERSION }, \
    { "MSG_ID_AIS_VESSEL", MAVLINK_MSG_ID_AIS_VESSEL }, \
    { "MSG_ID_UAVCAN_NODE_STATUS", MAVLINK_MSG_ID_UAVCAN_NODE_STATUS }, \
    { "MSG_ID_UAVCAN_NODE_INFO", MAVLINK_MSG_ID_UAVCAN_NODE_INFO }, \
    { "MSG_ID_PARAM_EXT_REQUEST_READ", MAVLINK_MSG_ID_PARAM_EXT_REQUEST_READ }, \
    { "MSG_ID_PARAM_EXT_REQUEST_LIST", MAVLINK_MSG_ID_PARAM_EXT_REQUEST_LIST }, \
    { "MSG_ID_PARAM_EXT_VALUE", MAVLINK_MSG_ID_PARAM_EXT_VALUE }, \
    { "MSG_ID_PARAM_EXT_SET", MAVLINK_MSG_ID_PARAM_EXT_SET }, \
    { "MSG_ID_PARAM_EXT_ACK", MAVLINK_MSG_ID_PARAM_EXT_ACK }, \
    { "MSG_ID_OBSTACLE_DISTANCE", MAVLINK_MSG_ID_OBSTACLE_DISTANCE }, \
    { "MSG_ID_ODOMETRY", MAVLINK_MSG_ID_ODOMETRY }, \
    { "MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS", MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_WAYPOINTS }, \
    { "MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER", MAVLINK_MSG_ID_TRAJECTORY_REPRESENTATION_BEZIER }, \
    { "MSG_ID_CELLULAR_STATUS", MAVLINK_MSG_ID_CELLULAR_STATUS }, \
    { "MSG_ID_ISBD_LINK_STATUS", MAVLINK_MSG_ID_ISBD_LINK_STATUS }, \
    { "MSG_ID_UTM_GLOBAL_POSITION", MAVLINK_MSG_ID_UTM_GLOBAL_POSITION }, \
    { "MSG_ID_DEBUG_FLOAT_ARRAY", MAVLINK_MSG_ID_DEBUG_FLOAT_ARRAY }, \
    { "MSG_ID_ORBIT_EXECUTION_STATUS", MAVLINK_MSG_ID_ORBIT_EXECUTION_STATUS }, \
    { "MSG_ID_SMART_BATTERY_INFO", MAVLINK_MSG_ID_SMART_BATTERY_INFO }, \
    { "MSG_ID_SMART_BATTERY_STATUS", MAVLINK_MSG_ID_SMART_BATTERY_STATUS }, \
    { "MSG_ID_ACTUATOR_OUTPUT_STATUS", MAVLINK_MSG_ID_ACTUATOR_OUTPUT_STATUS }, \
    { "MSG_ID_TIME_ESTIMATE_TO_TARGET", MAVLINK_MSG_ID_TIME_ESTIMATE_TO_TARGET }, \
    { "MSG_ID_TUNNEL", MAVLINK_MSG_ID_TUNNEL }, \
    { "MSG_ID_ONBOARD_COMPUTER_STATUS", MAVLINK_MSG_ID_ONBOARD_COMPUTER_STATUS }, \
    { "MSG_ID_COMPONENT_INFORMATION", MAVLINK_MSG_ID_COMPONENT_INFORMATION }, \
    { "MSG_ID_PLAY_TUNE_V2", MAVLINK_MSG_ID_PLAY_TUNE_V2 }, \
    { "MSG_ID_SUPPORTED_TUNES", MAVLINK_MSG_ID_SUPPORTED_TUNES }, \
    { "MSG_ID_WHEEL_DISTANCE", MAVLINK_MSG_ID_WHEEL_DISTANCE }, \
    \
    { "MAV_AUTOPILOT_GENERIC", 0 }, \
    { "MAV_AUTOPILOT_RESERVED", 1 }, \
    { "MAV_AUTOPILOT_SLUGS", 2 }, \
    { "MAV_AUTOPILOT_ARDUPILOTMEGA", 3 }, \
    { "MAV_AUTOPILOT_OPENPILOT", 4 }, \
    { "MAV_AUTOPILOT_GENERIC_WAYPOINTS_ONLY", 5 }, \
    { "MAV_AUTOPILOT_GENERIC_WAYPOINTS_AND_SIMPLE_NAVIGATION_ONLY", 6 }, \
    { "MAV_AUTOPILOT_GENERIC_MISSION_FULL", 7 }, \
    { "MAV_AUTOPILOT_INVALID", 8 }, \
    { "MAV_AUTOPILOT_PPZ", 9 }, \
    { "MAV_AUTOPILOT_UDB", 10 }, \
    { "MAV_AUTOPILOT_FP", 11 }, \
    { "MAV_AUTOPILOT_PX4", 12 }, \
    { "MAV_AUTOPILOT_SMACCMPILOT", 13 }, \
    { "MAV_AUTOPILOT_AUTOQUAD", 14 }, \
    { "MAV_AUTOPILOT_ARMAZILA", 15 }, \
    { "MAV_AUTOPILOT_AEROB", 16 }, \
    { "MAV_AUTOPILOT_ASLUAV", 17 }, \
    { "MAV_AUTOPILOT_SMARTAP", 18 }, \
    { "MAV_AUTOPILOT_AIRRAILS", 19 }, \
    \
    { "MAV_TYPE_GENERIC", 0 }, \
    { "MAV_TYPE_FIXED_WING", 1 }, \
    { "MAV_TYPE_QUADROTOR", 2 }, \
    { "MAV_TYPE_COAXIAL", 3 }, \
    { "MAV_TYPE_HELICOPTER", 4 }, \
    { "MAV_TYPE_ANTENNA_TRACKER", 5 }, \
    { "MAV_TYPE_GCS", 6 }, \
    { "MAV_TYPE_AIRSHIP", 7 }, \
    { "MAV_TYPE_FREE_BALLOON", 8 }, \
    { "MAV_TYPE_ROCKET", 9 }, \
    { "MAV_TYPE_GROUND_ROVER", 10 }, \
    { "MAV_TYPE_SURFACE_BOAT", 11 }, \
    { "MAV_TYPE_SUBMARINE", 12 }, \
    { "MAV_TYPE_HEXAROTOR", 13 }, \
    { "MAV_TYPE_OCTOROTOR", 14 }, \
    { "MAV_TYPE_TRICOPTER", 15 }, \
    { "MAV_TYPE_FLAPPING_WING", 16 }, \
    { "MAV_TYPE_KITE", 17 }, \
    { "MAV_TYPE_ONBOARD_CONTROLLER", 18 }, \
    { "MAV_TYPE_VTOL_DUOROTOR", 19 }, \
    { "MAV_TYPE_VTOL_QUADROTOR", 20 }, \
    { "MAV_TYPE_VTOL_TILTROTOR", 21 }, \
    { "MAV_TYPE_VTOL_RESERVED2", 22 }, \
    { "MAV_TYPE_VTOL_RESERVED3", 23 }, \
    { "MAV_TYPE_VTOL_RESERVED4", 24 }, \
    { "MAV_TYPE_VTOL_RESERVED5", 25 }, \
    { "MAV_TYPE_GIMBAL", 26 }, \
    { "MAV_TYPE_ADSB", 27 }, \
    { "MAV_TYPE_PARAFOIL", 28 }, \
    { "MAV_TYPE_DODECAROTOR", 29 }, \
    { "MAV_TYPE_CAMERA", 30 }, \
    { "MAV_TYPE_CHARGING_STATION", 31 }, \
    { "MAV_TYPE_FLARM", 32 }, \
    { "MAV_TYPE_SERVO", 33 }, \
    \
    { "FIRMWARE_VERSION_TYPE_DEV", 0 }, \
    { "FIRMWARE_VERSION_TYPE_ALPHA", 64 }, \
    { "FIRMWARE_VERSION_TYPE_BETA", 128 }, \
    { "FIRMWARE_VERSION_TYPE_RC", 192 }, \
    { "FIRMWARE_VERSION_TYPE_OFFICIAL", 255 }, \
    \
    { "HL_FAILURE_FLAG_GPS", 1 }, \
    { "HL_FAILURE_FLAG_DIFFERENTIAL_PRESSURE", 2 }, \
    { "HL_FAILURE_FLAG_ABSOLUTE_PRESSURE", 4 }, \
    { "HL_FAILURE_FLAG_3D_ACCEL", 8 }, \
    { "HL_FAILURE_FLAG_3D_GYRO", 16 }, \
    { "HL_FAILURE_FLAG_3D_MAG", 32 }, \
    { "HL_FAILURE_FLAG_TERRAIN", 64 }, \
    { "HL_FAILURE_FLAG_BATTERY", 128 }, \
    { "HL_FAILURE_FLAG_RC_RECEIVER", 256 }, \
    { "HL_FAILURE_FLAG_OFFBOARD_LINK", 512 }, \
    { "HL_FAILURE_FLAG_ENGINE", 1024 }, \
    { "HL_FAILURE_FLAG_GEOFENCE", 2048 }, \
    { "HL_FAILURE_FLAG_ESTIMATOR", 4096 }, \
    { "HL_FAILURE_FLAG_MISSION", 8192 }, \
    \
    { "MAV_MODE_FLAG_SAFETY_ARMED", 128 }, \
    { "MAV_MODE_FLAG_MANUAL_INPUT_ENABLED", 64 }, \
    { "MAV_MODE_FLAG_HIL_ENABLED", 32 }, \
    { "MAV_MODE_FLAG_STABILIZE_ENABLED", 16 }, \
    { "MAV_MODE_FLAG_GUIDED_ENABLED", 8 }, \
    { "MAV_MODE_FLAG_AUTO_ENABLED", 4 }, \
    { "MAV_MODE_FLAG_TEST_ENABLED", 2 }, \
    { "MAV_MODE_FLAG_CUSTOM_MODE_ENABLED", 1 }, \
    \
    { "MAV_MODE_FLAG_DECODE_POSITION_SAFETY", 128 }, \
    { "MAV_MODE_FLAG_DECODE_POSITION_MANUAL", 64 }, \
    { "MAV_MODE_FLAG_DECODE_POSITION_HIL", 32 }, \
    { "MAV_MODE_FLAG_DECODE_POSITION_STABILIZE", 16 }, \
    { "MAV_MODE_FLAG_DECODE_POSITION_GUIDED", 8 }, \
    { "MAV_MODE_FLAG_DECODE_POSITION_AUTO", 4 }, \
    { "MAV_MODE_FLAG_DECODE_POSITION_TEST", 2 }, \
    { "MAV_MODE_FLAG_DECODE_POSITION_CUSTOM_MODE", 1 }, \
    \
    { "MAV_GOTO_DO_HOLD", 0 }, \
    { "MAV_GOTO_DO_CONTINUE", 1 }, \
    { "MAV_GOTO_HOLD_AT_CURRENT_POSITION", 2 }, \
    { "MAV_GOTO_HOLD_AT_SPECIFIED_POSITION", 3 }, \
    \
    { "MAV_MODE_PREFLIGHT", 0 }, \
    { "MAV_MODE_STABILIZE_DISARMED", 80 }, \
    { "MAV_MODE_STABILIZE_ARMED", 208 }, \
    { "MAV_MODE_MANUAL_DISARMED", 64 }, \
    { "MAV_MODE_MANUAL_ARMED", 192 }, \
    { "MAV_MODE_GUIDED_DISARMED", 88 }, \
    { "MAV_MODE_GUIDED_ARMED", 216 }, \
    { "MAV_MODE_AUTO_DISARMED", 92 }, \
    { "MAV_MODE_AUTO_ARMED", 220 }, \
    { "MAV_MODE_TEST_DISARMED", 66 }, \
    { "MAV_MODE_TEST_ARMED", 194 }, \
    \
    { "MAV_STATE_UNINIT", 0 }, \
    { "MAV_STATE_BOOT", 1 }, \
    { "MAV_STATE_CALIBRATING", 2 }, \
    { "MAV_STATE_STANDBY", 3 }, \
    { "MAV_STATE_ACTIVE", 4 }, \
    { "MAV_STATE_CRITICAL", 5 }, \
    { "MAV_STATE_EMERGENCY", 6 }, \
    { "MAV_STATE_POWEROFF", 7 }, \
    { "MAV_STATE_FLIGHT_TERMINATION", 8 }, \
    \
    { "MAV_COMP_ID_ALL", 0 }, \
    { "MAV_COMP_ID_AUTOPILOT1", 1 }, \
    { "MAV_COMP_ID_USER1", 25 }, \
    { "MAV_COMP_ID_USER2", 26 }, \
    { "MAV_COMP_ID_USER3", 27 }, \
    { "MAV_COMP_ID_USER4", 28 }, \
    { "MAV_COMP_ID_USER5", 29 }, \
    { "MAV_COMP_ID_USER6", 30 }, \
    { "MAV_COMP_ID_USER7", 31 }, \
    { "MAV_COMP_ID_USER8", 32 }, \
    { "MAV_COMP_ID_USER9", 33 }, \
    { "MAV_COMP_ID_USER10", 34 }, \
    { "MAV_COMP_ID_USER11", 35 }, \
    { "MAV_COMP_ID_USER12", 36 }, \
    { "MAV_COMP_ID_USER13", 37 }, \
    { "MAV_COMP_ID_USER14", 38 }, \
    { "MAV_COMP_ID_USER15", 39 }, \
    { "MAV_COMP_ID_USER16", 40 }, \
    { "MAV_COMP_ID_USER17", 41 }, \
    { "MAV_COMP_ID_USER18", 42 }, \
    { "MAV_COMP_ID_USER19", 43 }, \
    { "MAV_COMP_ID_USER20", 44 }, \
    { "MAV_COMP_ID_USER21", 45 }, \
    { "MAV_COMP_ID_USER22", 46 }, \
    { "MAV_COMP_ID_USER23", 47 }, \
    { "MAV_COMP_ID_USER24", 48 }, \
    { "MAV_COMP_ID_USER25", 49 }, \
    { "MAV_COMP_ID_USER26", 50 }, \
    { "MAV_COMP_ID_USER27", 51 }, \
    { "MAV_COMP_ID_USER28", 52 }, \
    { "MAV_COMP_ID_USER29", 53 }, \
    { "MAV_COMP_ID_USER30", 54 }, \
    { "MAV_COMP_ID_USER31", 55 }, \
    { "MAV_COMP_ID_USER32", 56 }, \
    { "MAV_COMP_ID_USER33", 57 }, \
    { "MAV_COMP_ID_USER34", 58 }, \
    { "MAV_COMP_ID_USER35", 59 }, \
    { "MAV_COMP_ID_USER36", 60 }, \
    { "MAV_COMP_ID_USER37", 61 }, \
    { "MAV_COMP_ID_USER38", 62 }, \
    { "MAV_COMP_ID_USER39", 63 }, \
    { "MAV_COMP_ID_USER40", 64 }, \
    { "MAV_COMP_ID_USER41", 65 }, \
    { "MAV_COMP_ID_USER42", 66 }, \
    { "MAV_COMP_ID_USER43", 67 }, \
    { "MAV_COMP_ID_USER44", 68 }, \
    { "MAV_COMP_ID_USER45", 69 }, \
    { "MAV_COMP_ID_USER46", 70 }, \
    { "MAV_COMP_ID_USER47", 71 }, \
    { "MAV_COMP_ID_USER48", 72 }, \
    { "MAV_COMP_ID_USER49", 73 }, \
    { "MAV_COMP_ID_USER50", 74 }, \
    { "MAV_COMP_ID_USER51", 75 }, \
    { "MAV_COMP_ID_USER52", 76 }, \
    { "MAV_COMP_ID_USER53", 77 }, \
    { "MAV_COMP_ID_USER54", 78 }, \
    { "MAV_COMP_ID_USER55", 79 }, \
    { "MAV_COMP_ID_USER56", 80 }, \
    { "MAV_COMP_ID_USER57", 81 }, \
    { "MAV_COMP_ID_USER58", 82 }, \
    { "MAV_COMP_ID_USER59", 83 }, \
    { "MAV_COMP_ID_USER60", 84 }, \
    { "MAV_COMP_ID_USER61", 85 }, \
    { "MAV_COMP_ID_USER62", 86 }, \
    { "MAV_COMP_ID_USER63", 87 }, \
    { "MAV_COMP_ID_USER64", 88 }, \
    { "MAV_COMP_ID_USER65", 89 }, \
    { "MAV_COMP_ID_USER66", 90 }, \
    { "MAV_COMP_ID_USER67", 91 }, \
    { "MAV_COMP_ID_USER68", 92 }, \
    { "MAV_COMP_ID_USER69", 93 }, \
    { "MAV_COMP_ID_USER70", 94 }, \
    { "MAV_COMP_ID_USER71", 95 }, \
    { "MAV_COMP_ID_USER72", 96 }, \
    { "MAV_COMP_ID_USER73", 97 }, \
    { "MAV_COMP_ID_USER74", 98 }, \
    { "MAV_COMP_ID_USER75", 99 }, \
    { "MAV_COMP_ID_CAMERA", 100 }, \
    { "MAV_COMP_ID_CAMERA2", 101 }, \
    { "MAV_COMP_ID_CAMERA3", 102 }, \
    { "MAV_COMP_ID_CAMERA4", 103 }, \
    { "MAV_COMP_ID_CAMERA5", 104 }, \
    { "MAV_COMP_ID_CAMERA6", 105 }, \
    { "MAV_COMP_ID_SERVO1", 140 }, \
    { "MAV_COMP_ID_SERVO2", 141 }, \
    { "MAV_COMP_ID_SERVO3", 142 }, \
    { "MAV_COMP_ID_SERVO4", 143 }, \
    { "MAV_COMP_ID_SERVO5", 144 }, \
    { "MAV_COMP_ID_SERVO6", 145 }, \
    { "MAV_COMP_ID_SERVO7", 146 }, \
    { "MAV_COMP_ID_SERVO8", 147 }, \
    { "MAV_COMP_ID_SERVO9", 148 }, \
    { "MAV_COMP_ID_SERVO10", 149 }, \
    { "MAV_COMP_ID_SERVO11", 150 }, \
    { "MAV_COMP_ID_SERVO12", 151 }, \
    { "MAV_COMP_ID_SERVO13", 152 }, \
    { "MAV_COMP_ID_SERVO14", 153 }, \
    { "MAV_COMP_ID_GIMBAL", 154 }, \
    { "MAV_COMP_ID_LOG", 155 }, \
    { "MAV_COMP_ID_ADSB", 156 }, \
    { "MAV_COMP_ID_OSD", 157 }, \
    { "MAV_COMP_ID_PERIPHERAL", 158 }, \
    { "MAV_COMP_ID_QX1_GIMBAL", 159 }, \
    { "MAV_COMP_ID_FLARM", 160 }, \
    { "MAV_COMP_ID_GIMBAL2", 171 }, \
    { "MAV_COMP_ID_GIMBAL3", 172 }, \
    { "MAV_COMP_ID_GIMBAL4", 173 }, \
    { "MAV_COMP_ID_GIMBAL5", 174 }, \
    { "MAV_COMP_ID_GIMBAL6", 175 }, \
    { "MAV_COMP_ID_MISSIONPLANNER", 190 }, \
    { "MAV_COMP_ID_PATHPLANNER", 195 }, \
    { "MAV_COMP_ID_OBSTACLE_AVOIDANCE", 196 }, \
    { "MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY", 197 }, \
    { "MAV_COMP_ID_PAIRING_MANAGER", 198 }, \
    { "MAV_COMP_ID_IMU", 200 }, \
    { "MAV_COMP_ID_IMU_2", 201 }, \
    { "MAV_COMP_ID_IMU_3", 202 }, \
    { "MAV_COMP_ID_GPS", 220 }, \
    { "MAV_COMP_ID_GPS2", 221 }, \
    { "MAV_COMP_ID_UDP_BRIDGE", 240 }, \
    { "MAV_COMP_ID_UART_BRIDGE", 241 }, \
    { "MAV_COMP_ID_TUNNEL_NODE", 242 }, \
    { "MAV_COMP_ID_SYSTEM_CONTROL", 250 }, \
    \
    { "MAV_SYS_STATUS_SENSOR_3D_GYRO", 1 }, \
    { "MAV_SYS_STATUS_SENSOR_3D_ACCEL", 2 }, \
    { "MAV_SYS_STATUS_SENSOR_3D_MAG", 4 }, \
    { "MAV_SYS_STATUS_SENSOR_ABSOLUTE_PRESSURE", 8 }, \
    { "MAV_SYS_STATUS_SENSOR_DIFFERENTIAL_PRESSURE", 16 }, \
    { "MAV_SYS_STATUS_SENSOR_GPS", 32 }, \
    { "MAV_SYS_STATUS_SENSOR_OPTICAL_FLOW", 64 }, \
    { "MAV_SYS_STATUS_SENSOR_VISION_POSITION", 128 }, \
    { "MAV_SYS_STATUS_SENSOR_LASER_POSITION", 256 }, \
    { "MAV_SYS_STATUS_SENSOR_EXTERNAL_GROUND_TRUTH", 512 }, \
    { "MAV_SYS_STATUS_SENSOR_ANGULAR_RATE_CONTROL", 1024 }, \
    { "MAV_SYS_STATUS_SENSOR_ATTITUDE_STABILIZATION", 2048 }, \
    { "MAV_SYS_STATUS_SENSOR_YAW_POSITION", 4096 }, \
    { "MAV_SYS_STATUS_SENSOR_Z_ALTITUDE_CONTROL", 8192 }, \
    { "MAV_SYS_STATUS_SENSOR_XY_POSITION_CONTROL", 16384 }, \
    { "MAV_SYS_STATUS_SENSOR_MOTOR_OUTPUTS", 32768 }, \
    { "MAV_SYS_STATUS_SENSOR_RC_RECEIVER", 65536 }, \
    { "MAV_SYS_STATUS_SENSOR_3D_GYRO2", 131072 }, \
    { "MAV_SYS_STATUS_SENSOR_3D_ACCEL2", 262144 }, \
    { "MAV_SYS_STATUS_SENSOR_3D_MAG2", 524288 }, \
    { "MAV_SYS_STATUS_GEOFENCE", 1048576 }, \
    { "MAV_SYS_STATUS_AHRS", 2097152 }, \
    { "MAV_SYS_STATUS_TERRAIN", 4194304 }, \
    { "MAV_SYS_STATUS_REVERSE_MOTOR", 8388608 }, \
    { "MAV_SYS_STATUS_LOGGING", 16777216 }, \
    { "MAV_SYS_STATUS_SENSOR_BATTERY", 33554432 }, \
    { "MAV_SYS_STATUS_SENSOR_PROXIMITY", 67108864 }, \
    { "MAV_SYS_STATUS_SENSOR_SATCOM", 134217728 }, \
    { "MAV_SYS_STATUS_PREARM_CHECK", 268435456 }, \
    \
    { "MAV_FRAME_GLOBAL", 0 }, \
    { "MAV_FRAME_LOCAL_NED", 1 }, \
    { "MAV_FRAME_MISSION", 2 }, \
    { "MAV_FRAME_GLOBAL_RELATIVE_ALT", 3 }, \
    { "MAV_FRAME_LOCAL_ENU", 4 }, \
    { "MAV_FRAME_GLOBAL_INT", 5 }, \
    { "MAV_FRAME_GLOBAL_RELATIVE_ALT_INT", 6 }, \
    { "MAV_FRAME_LOCAL_OFFSET_NED", 7 }, \
    { "MAV_FRAME_BODY_NED", 8 }, \
    { "MAV_FRAME_BODY_OFFSET_NED", 9 }, \
    { "MAV_FRAME_GLOBAL_TERRAIN_ALT", 10 }, \
    { "MAV_FRAME_GLOBAL_TERRAIN_ALT_INT", 11 }, \
    { "MAV_FRAME_BODY_FRD", 12 }, \
    { "MAV_FRAME_BODY_FLU", 13 }, \
    { "MAV_FRAME_MOCAP_NED", 14 }, \
    { "MAV_FRAME_MOCAP_ENU", 15 }, \
    { "MAV_FRAME_VISION_NED", 16 }, \
    { "MAV_FRAME_VISION_ENU", 17 }, \
    { "MAV_FRAME_ESTIM_NED", 18 }, \
    { "MAV_FRAME_ESTIM_ENU", 19 }, \
    { "MAV_FRAME_LOCAL_FRD", 20 }, \
    { "MAV_FRAME_LOCAL_FLU", 21 }, \
    \
    { "MAVLINK_DATA_STREAM_IMG_JPEG", 22 }, \
    { "MAVLINK_DATA_STREAM_IMG_BMP", 23 }, \
    { "MAVLINK_DATA_STREAM_IMG_RAW8U", 24 }, \
    { "MAVLINK_DATA_STREAM_IMG_RAW32U", 25 }, \
    { "MAVLINK_DATA_STREAM_IMG_PGM", 26 }, \
    { "MAVLINK_DATA_STREAM_IMG_PNG", 27 }, \
    \
    { "FENCE_ACTION_NONE", 0 }, \
    { "FENCE_ACTION_GUIDED", 1 }, \
    { "FENCE_ACTION_REPORT", 2 }, \
    { "FENCE_ACTION_GUIDED_THR_PASS", 3 }, \
    { "FENCE_ACTION_RTL", 4 }, \
    \
    { "FENCE_BREACH_NONE", 0 }, \
    { "FENCE_BREACH_MINALT", 1 }, \
    { "FENCE_BREACH_MAXALT", 2 }, \
    { "FENCE_BREACH_BOUNDARY", 3 }, \
    \
    { "FENCE_MITIGATE_UNKNOWN", 0 }, \
    { "FENCE_MITIGATE_NONE", 1 }, \
    { "FENCE_MITIGATE_VEL_LIMIT", 2 }, \
    \
    { "MAV_MOUNT_MODE_RETRACT", 0 }, \
    { "MAV_MOUNT_MODE_NEUTRAL", 1 }, \
    { "MAV_MOUNT_MODE_MAVLINK_TARGETING", 2 }, \
    { "MAV_MOUNT_MODE_RC_TARGETING", 3 }, \
    { "MAV_MOUNT_MODE_GPS_POINT", 4 }, \
    { "MAV_MOUNT_MODE_SYSID_TARGET", 5 }, \
    \
    { "UAVCAN_NODE_HEALTH_OK", 0 }, \
    { "UAVCAN_NODE_HEALTH_WARNING", 1 }, \
    { "UAVCAN_NODE_HEALTH_ERROR", 2 }, \
    { "UAVCAN_NODE_HEALTH_CRITICAL", 3 }, \
    \
    { "UAVCAN_NODE_MODE_OPERATIONAL", 0 }, \
    { "UAVCAN_NODE_MODE_INITIALIZATION", 1 }, \
    { "UAVCAN_NODE_MODE_MAINTENANCE", 2 }, \
    { "UAVCAN_NODE_MODE_SOFTWARE_UPDATE", 3 }, \
    { "UAVCAN_NODE_MODE_OFFLINE", 7 }, \
    \
    { "STORAGE_STATUS_EMPTY", 0 }, \
    { "STORAGE_STATUS_UNFORMATTED", 1 }, \
    { "STORAGE_STATUS_READY", 2 }, \
    { "STORAGE_STATUS_NOT_SUPPORTED", 3 }, \
    \
    { "ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TO_CIRCLE_CENTER", 0 }, \
    { "ORBIT_YAW_BEHAVIOUR_HOLD_INITIAL_HEADING", 1 }, \
    { "ORBIT_YAW_BEHAVIOUR_UNCONTROLLED", 2 }, \
    { "ORBIT_YAW_BEHAVIOUR_HOLD_FRONT_TANGENT_TO_CIRCLE", 3 }, \
    { "ORBIT_YAW_BEHAVIOUR_RC_CONTROLLED", 4 }, \
    \
    { "MAV_CMD_NAV_WAYPOINT", 16 }, \
    { "MAV_CMD_NAV_LOITER_UNLIM", 17 }, \
    { "MAV_CMD_NAV_LOITER_TURNS", 18 }, \
    { "MAV_CMD_NAV_LOITER_TIME", 19 }, \
    { "MAV_CMD_NAV_RETURN_TO_LAUNCH", 20 }, \
    { "MAV_CMD_NAV_LAND", 21 }, \
    { "MAV_CMD_NAV_TAKEOFF", 22 }, \
    { "MAV_CMD_NAV_LAND_LOCAL", 23 }, \
    { "MAV_CMD_NAV_TAKEOFF_LOCAL", 24 }, \
    { "MAV_CMD_NAV_FOLLOW", 25 }, \
    { "MAV_CMD_NAV_CONTINUE_AND_CHANGE_ALT", 30 }, \
    { "MAV_CMD_NAV_LOITER_TO_ALT", 31 }, \
    { "MAV_CMD_DO_FOLLOW", 32 }, \
    { "MAV_CMD_DO_FOLLOW_REPOSITION", 33 }, \
    { "MAV_CMD_DO_ORBIT", 34 }, \
    { "MAV_CMD_NAV_ROI", 80 }, \
    { "MAV_CMD_NAV_PATHPLANNING", 81 }, \
    { "MAV_CMD_NAV_SPLINE_WAYPOINT", 82 }, \
    { "MAV_CMD_NAV_VTOL_TAKEOFF", 84 }, \
    { "MAV_CMD_NAV_VTOL_LAND", 85 }, \
    { "MAV_CMD_NAV_GUIDED_ENABLE", 92 }, \
    { "MAV_CMD_NAV_DELAY", 93 }, \
    { "MAV_CMD_NAV_PAYLOAD_PLACE", 94 }, \
    { "MAV_CMD_NAV_LAST", 95 }, \
    { "MAV_CMD_CONDITION_DELAY", 112 }, \
    { "MAV_CMD_CONDITION_CHANGE_ALT", 113 }, \
    { "MAV_CMD_CONDITION_DISTANCE", 114 }, \
    { "MAV_CMD_CONDITION_YAW", 115 }, \
    { "MAV_CMD_CONDITION_LAST", 159 }, \
    { "MAV_CMD_DO_SET_MODE", 176 }, \
    { "MAV_CMD_DO_JUMP", 177 }, \
    { "MAV_CMD_DO_CHANGE_SPEED", 178 }, \
    { "MAV_CMD_DO_SET_HOME", 179 }, \
    { "MAV_CMD_DO_SET_PARAMETER", 180 }, \
    { "MAV_CMD_DO_SET_RELAY", 181 }, \
    { "MAV_CMD_DO_REPEAT_RELAY", 182 }, \
    { "MAV_CMD_DO_SET_SERVO", 183 }, \
    { "MAV_CMD_DO_REPEAT_SERVO", 184 }, \
    { "MAV_CMD_DO_FLIGHTTERMINATION", 185 }, \
    { "MAV_CMD_DO_CHANGE_ALTITUDE", 186 }, \
    { "MAV_CMD_DO_LAND_START", 189 }, \
    { "MAV_CMD_DO_RALLY_LAND", 190 }, \
    { "MAV_CMD_DO_GO_AROUND", 191 }, \
    { "MAV_CMD_DO_REPOSITION", 192 }, \
    { "MAV_CMD_DO_PAUSE_CONTINUE", 193 }, \
    { "MAV_CMD_DO_SET_REVERSE", 194 }, \
    { "MAV_CMD_DO_SET_ROI_LOCATION", 195 }, \
    { "MAV_CMD_DO_SET_ROI_WPNEXT_OFFSET", 196 }, \
    { "MAV_CMD_DO_SET_ROI_NONE", 197 }, \
    { "MAV_CMD_DO_SET_ROI_SYSID", 198 }, \
    { "MAV_CMD_DO_CONTROL_VIDEO", 200 }, \
    { "MAV_CMD_DO_SET_ROI", 201 }, \
    { "MAV_CMD_DO_DIGICAM_CONFIGURE", 202 }, \
    { "MAV_CMD_DO_DIGICAM_CONTROL", 203 }, \
    { "MAV_CMD_DO_MOUNT_CONFIGURE", 204 }, \
    { "MAV_CMD_DO_MOUNT_CONTROL", 205 }, \
    { "MAV_CMD_DO_SET_CAM_TRIGG_DIST", 206 }, \
    { "MAV_CMD_DO_FENCE_ENABLE", 207 }, \
    { "MAV_CMD_DO_PARACHUTE", 208 }, \
    { "MAV_CMD_DO_MOTOR_TEST", 209 }, \
    { "MAV_CMD_DO_INVERTED_FLIGHT", 210 }, \
    { "MAV_CMD_NAV_SET_YAW_SPEED", 213 }, \
    { "MAV_CMD_DO_SET_CAM_TRIGG_INTERVAL", 214 }, \
    { "MAV_CMD_DO_MOUNT_CONTROL_QUAT", 220 }, \
    { "MAV_CMD_DO_GUIDED_MASTER", 221 }, \
    { "MAV_CMD_DO_GUIDED_LIMITS", 222 }, \
    { "MAV_CMD_DO_ENGINE_CONTROL", 223 }, \
    { "MAV_CMD_DO_SET_MISSION_CURRENT", 224 }, \
    { "MAV_CMD_DO_LAST", 240 }, \
    { "MAV_CMD_PREFLIGHT_CALIBRATION", 241 }, \
    { "MAV_CMD_PREFLIGHT_SET_SENSOR_OFFSETS", 242 }, \
    { "MAV_CMD_PREFLIGHT_UAVCAN", 243 }, \
    { "MAV_CMD_PREFLIGHT_STORAGE", 245 }, \
    { "MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN", 246 }, \
    { "MAV_CMD_OVERRIDE_GOTO", 252 }, \
    { "MAV_CMD_MISSION_START", 300 }, \
    { "MAV_CMD_COMPONENT_ARM_DISARM", 400 }, \
    { "MAV_CMD_ILLUMINATOR_ON_OFF", 405 }, \
    { "MAV_CMD_GET_HOME_POSITION", 410 }, \
    { "MAV_CMD_START_RX_PAIR", 500 }, \
    { "MAV_CMD_GET_MESSAGE_INTERVAL", 510 }, \
    { "MAV_CMD_SET_MESSAGE_INTERVAL", 511 }, \
    { "MAV_CMD_REQUEST_MESSAGE", 512 }, \
    { "MAV_CMD_REQUEST_PROTOCOL_VERSION", 519 }, \
    { "MAV_CMD_REQUEST_AUTOPILOT_CAPABILITIES", 520 }, \
    { "MAV_CMD_REQUEST_CAMERA_INFORMATION", 521 }, \
    { "MAV_CMD_REQUEST_CAMERA_SETTINGS", 522 }, \
    { "MAV_CMD_REQUEST_STORAGE_INFORMATION", 525 }, \
    { "MAV_CMD_STORAGE_FORMAT", 526 }, \
    { "MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS", 527 }, \
    { "MAV_CMD_REQUEST_FLIGHT_INFORMATION", 528 }, \
    { "MAV_CMD_RESET_CAMERA_SETTINGS", 529 }, \
    { "MAV_CMD_SET_CAMERA_MODE", 530 }, \
    { "MAV_CMD_SET_CAMERA_ZOOM", 531 }, \
    { "MAV_CMD_SET_CAMERA_FOCUS", 532 }, \
    { "MAV_CMD_JUMP_TAG", 600 }, \
    { "MAV_CMD_DO_JUMP_TAG", 601 }, \
    { "MAV_CMD_IMAGE_START_CAPTURE", 2000 }, \
    { "MAV_CMD_IMAGE_STOP_CAPTURE", 2001 }, \
    { "MAV_CMD_REQUEST_CAMERA_IMAGE_CAPTURE", 2002 }, \
    { "MAV_CMD_DO_TRIGGER_CONTROL", 2003 }, \
    { "MAV_CMD_VIDEO_START_CAPTURE", 2500 }, \
    { "MAV_CMD_VIDEO_STOP_CAPTURE", 2501 }, \
    { "MAV_CMD_VIDEO_START_STREAMING", 2502 }, \
    { "MAV_CMD_VIDEO_STOP_STREAMING", 2503 }, \
    { "MAV_CMD_REQUEST_VIDEO_STREAM_INFORMATION", 2504 }, \
    { "MAV_CMD_REQUEST_VIDEO_STREAM_STATUS", 2505 }, \
    { "MAV_CMD_LOGGING_START", 2510 }, \
    { "MAV_CMD_LOGGING_STOP", 2511 }, \
    { "MAV_CMD_AIRFRAME_CONFIGURATION", 2520 }, \
    { "MAV_CMD_CONTROL_HIGH_LATENCY", 2600 }, \
    { "MAV_CMD_PANORAMA_CREATE", 2800 }, \
    { "MAV_CMD_DO_VTOL_TRANSITION", 3000 }, \
    { "MAV_CMD_ARM_AUTHORIZATION_REQUEST", 3001 }, \
    { "MAV_CMD_SET_GUIDED_SUBMODE_STANDARD", 4000 }, \
    { "MAV_CMD_SET_GUIDED_SUBMODE_CIRCLE", 4001 }, \
    { "MAV_CMD_CONDITION_GATE", 4501 }, \
    { "MAV_CMD_NAV_FENCE_RETURN_POINT", 5000 }, \
    { "MAV_CMD_NAV_FENCE_POLYGON_VERTEX_INCLUSION", 5001 }, \
    { "MAV_CMD_NAV_FENCE_POLYGON_VERTEX_EXCLUSION", 5002 }, \
    { "MAV_CMD_NAV_FENCE_CIRCLE_INCLUSION", 5003 }, \
    { "MAV_CMD_NAV_FENCE_CIRCLE_EXCLUSION", 5004 }, \
    { "MAV_CMD_NAV_RALLY_POINT", 5100 }, \
    { "MAV_CMD_UAVCAN_GET_NODE_INFO", 5200 }, \
    { "MAV_CMD_PAYLOAD_PREPARE_DEPLOY", 30001 }, \
    { "MAV_CMD_PAYLOAD_CONTROL_DEPLOY", 30002 }, \
    { "MAV_CMD_WAYPOINT_USER_1", 31000 }, \
    { "MAV_CMD_WAYPOINT_USER_2", 31001 }, \
    { "MAV_CMD_WAYPOINT_USER_3", 31002 }, \
    { "MAV_CMD_WAYPOINT_USER_4", 31003 }, \
    { "MAV_CMD_WAYPOINT_USER_5", 31004 }, \
    { "MAV_CMD_SPATIAL_USER_1", 31005 }, \
    { "MAV_CMD_SPATIAL_USER_2", 31006 }, \
    { "MAV_CMD_SPATIAL_USER_3", 31007 }, \
    { "MAV_CMD_SPATIAL_USER_4", 31008 }, \
    { "MAV_CMD_SPATIAL_USER_5", 31009 }, \
    { "MAV_CMD_USER_1", 31010 }, \
    { "MAV_CMD_USER_2", 31011 }, \
    { "MAV_CMD_USER_3", 31012 }, \
    { "MAV_CMD_USER_4", 31013 }, \
    { "MAV_CMD_USER_5", 31014 }, \
    \
    { "MAV_DATA_STREAM_ALL", 0 }, \
    { "MAV_DATA_STREAM_RAW_SENSORS", 1 }, \
    { "MAV_DATA_STREAM_EXTENDED_STATUS", 2 }, \
    { "MAV_DATA_STREAM_RC_CHANNELS", 3 }, \
    { "MAV_DATA_STREAM_RAW_CONTROLLER", 4 }, \
    { "MAV_DATA_STREAM_POSITION", 6 }, \
    { "MAV_DATA_STREAM_EXTRA1", 10 }, \
    { "MAV_DATA_STREAM_EXTRA2", 11 }, \
    { "MAV_DATA_STREAM_EXTRA3", 12 }, \
    \
    { "MAV_ROI_NONE", 0 }, \
    { "MAV_ROI_WPNEXT", 1 }, \
    { "MAV_ROI_WPINDEX", 2 }, \
    { "MAV_ROI_LOCATION", 3 }, \
    { "MAV_ROI_TARGET", 4 }, \
    \
    { "MAV_CMD_ACK_OK", 5 }, \
    { "MAV_CMD_ACK_ERR_FAIL", 6 }, \
    { "MAV_CMD_ACK_ERR_ACCESS_DENIED", 7 }, \
    { "MAV_CMD_ACK_ERR_NOT_SUPPORTED", 8 }, \
    { "MAV_CMD_ACK_ERR_COORDINATE_FRAME_NOT_SUPPORTED", 9 }, \
    { "MAV_CMD_ACK_ERR_COORDINATES_OUT_OF_RANGE", 10 }, \
    { "MAV_CMD_ACK_ERR_X_LAT_OUT_OF_RANGE", 11 }, \
    { "MAV_CMD_ACK_ERR_Y_LON_OUT_OF_RANGE", 12 }, \
    { "MAV_CMD_ACK_ERR_Z_ALT_OUT_OF_RANGE", 13 }, \
    \
    { "MAV_PARAM_TYPE_UINT8", 1 }, \
    { "MAV_PARAM_TYPE_INT8", 2 }, \
    { "MAV_PARAM_TYPE_UINT16", 3 }, \
    { "MAV_PARAM_TYPE_INT16", 4 }, \
    { "MAV_PARAM_TYPE_UINT32", 5 }, \
    { "MAV_PARAM_TYPE_INT32", 6 }, \
    { "MAV_PARAM_TYPE_UINT64", 7 }, \
    { "MAV_PARAM_TYPE_INT64", 8 }, \
    { "MAV_PARAM_TYPE_REAL32", 9 }, \
    { "MAV_PARAM_TYPE_REAL64", 10 }, \
    \
    { "MAV_PARAM_EXT_TYPE_UINT8", 1 }, \
    { "MAV_PARAM_EXT_TYPE_INT8", 2 }, \
    { "MAV_PARAM_EXT_TYPE_UINT16", 3 }, \
    { "MAV_PARAM_EXT_TYPE_INT16", 4 }, \
    { "MAV_PARAM_EXT_TYPE_UINT32", 5 }, \
    { "MAV_PARAM_EXT_TYPE_INT32", 6 }, \
    { "MAV_PARAM_EXT_TYPE_UINT64", 7 }, \
    { "MAV_PARAM_EXT_TYPE_INT64", 8 }, \
    { "MAV_PARAM_EXT_TYPE_REAL32", 9 }, \
    { "MAV_PARAM_EXT_TYPE_REAL64", 10 }, \
    { "MAV_PARAM_EXT_TYPE_CUSTOM", 11 }, \
    \
    { "MAV_RESULT_ACCEPTED", 0 }, \
    { "MAV_RESULT_TEMPORARILY_REJECTED", 1 }, \
    { "MAV_RESULT_DENIED", 2 }, \
    { "MAV_RESULT_UNSUPPORTED", 3 }, \
    { "MAV_RESULT_FAILED", 4 }, \
    { "MAV_RESULT_IN_PROGRESS", 5 }, \
    \
    { "MAV_MISSION_ACCEPTED", 0 }, \
    { "MAV_MISSION_ERROR", 1 }, \
    { "MAV_MISSION_UNSUPPORTED_FRAME", 2 }, \
    { "MAV_MISSION_UNSUPPORTED", 3 }, \
    { "MAV_MISSION_NO_SPACE", 4 }, \
    { "MAV_MISSION_INVALID", 5 }, \
    { "MAV_MISSION_INVALID_PARAM1", 6 }, \
    { "MAV_MISSION_INVALID_PARAM2", 7 }, \
    { "MAV_MISSION_INVALID_PARAM3", 8 }, \
    { "MAV_MISSION_INVALID_PARAM4", 9 }, \
    { "MAV_MISSION_INVALID_PARAM5_X", 10 }, \
    { "MAV_MISSION_INVALID_PARAM6_Y", 11 }, \
    { "MAV_MISSION_INVALID_PARAM7", 12 }, \
    { "MAV_MISSION_INVALID_SEQUENCE", 13 }, \
    { "MAV_MISSION_DENIED", 14 }, \
    { "MAV_MISSION_OPERATION_CANCELLED", 15 }, \
    \
    { "MAV_SEVERITY_EMERGENCY", 0 }, \
    { "MAV_SEVERITY_ALERT", 1 }, \
    { "MAV_SEVERITY_CRITICAL", 2 }, \
    { "MAV_SEVERITY_ERROR", 3 }, \
    { "MAV_SEVERITY_WARNING", 4 }, \
    { "MAV_SEVERITY_NOTICE", 5 }, \
    { "MAV_SEVERITY_INFO", 6 }, \
    { "MAV_SEVERITY_DEBUG", 7 }, \
    \
    { "MAV_POWER_STATUS_BRICK_VALID", 1 }, \
    { "MAV_POWER_STATUS_SERVO_VALID", 2 }, \
    { "MAV_POWER_STATUS_USB_CONNECTED", 4 }, \
    { "MAV_POWER_STATUS_PERIPH_OVERCURRENT", 8 }, \
    { "MAV_POWER_STATUS_PERIPH_HIPOWER_OVERCURRENT", 16 }, \
    { "MAV_POWER_STATUS_CHANGED", 32 }, \
    \
    { "SERIAL_CONTROL_DEV_TELEM1", 0 }, \
    { "SERIAL_CONTROL_DEV_TELEM2", 1 }, \
    { "SERIAL_CONTROL_DEV_GPS1", 2 }, \
    { "SERIAL_CONTROL_DEV_GPS2", 3 }, \
    { "SERIAL_CONTROL_DEV_SHELL", 10 }, \
    { "SERIAL_CONTROL_SERIAL0", 100 }, \
    { "SERIAL_CONTROL_SERIAL1", 101 }, \
    { "SERIAL_CONTROL_SERIAL2", 102 }, \
    { "SERIAL_CONTROL_SERIAL3", 103 }, \
    { "SERIAL_CONTROL_SERIAL4", 104 }, \
    { "SERIAL_CONTROL_SERIAL5", 105 }, \
    { "SERIAL_CONTROL_SERIAL6", 106 }, \
    { "SERIAL_CONTROL_SERIAL7", 107 }, \
    { "SERIAL_CONTROL_SERIAL8", 108 }, \
    { "SERIAL_CONTROL_SERIAL9", 109 }, \
    \
    { "SERIAL_CONTROL_FLAG_REPLY", 1 }, \
    { "SERIAL_CONTROL_FLAG_RESPOND", 2 }, \
    { "SERIAL_CONTROL_FLAG_EXCLUSIVE", 4 }, \
    { "SERIAL_CONTROL_FLAG_BLOCKING", 8 }, \
    { "SERIAL_CONTROL_FLAG_MULTI", 16 }, \
    \
    { "MAV_DISTANCE_SENSOR_LASER", 0 }, \
    { "MAV_DISTANCE_SENSOR_ULTRASOUND", 1 }, \
    { "MAV_DISTANCE_SENSOR_INFRARED", 2 }, \
    { "MAV_DISTANCE_SENSOR_RADAR", 3 }, \
    { "MAV_DISTANCE_SENSOR_UNKNOWN", 4 }, \
    \
    { "MAV_SENSOR_ROTATION_NONE", 0 }, \
    { "MAV_SENSOR_ROTATION_YAW_45", 1 }, \
    { "MAV_SENSOR_ROTATION_YAW_90", 2 }, \
    { "MAV_SENSOR_ROTATION_YAW_135", 3 }, \
    { "MAV_SENSOR_ROTATION_YAW_180", 4 }, \
    { "MAV_SENSOR_ROTATION_YAW_225", 5 }, \
    { "MAV_SENSOR_ROTATION_YAW_270", 6 }, \
    { "MAV_SENSOR_ROTATION_YAW_315", 7 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180", 8 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180_YAW_45", 9 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180_YAW_90", 10 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180_YAW_135", 11 }, \
    { "MAV_SENSOR_ROTATION_PITCH_180", 12 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180_YAW_225", 13 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180_YAW_270", 14 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180_YAW_315", 15 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90", 16 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_YAW_45", 17 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_YAW_90", 18 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_YAW_135", 19 }, \
    { "MAV_SENSOR_ROTATION_ROLL_270", 20 }, \
    { "MAV_SENSOR_ROTATION_ROLL_270_YAW_45", 21 }, \
    { "MAV_SENSOR_ROTATION_ROLL_270_YAW_90", 22 }, \
    { "MAV_SENSOR_ROTATION_ROLL_270_YAW_135", 23 }, \
    { "MAV_SENSOR_ROTATION_PITCH_90", 24 }, \
    { "MAV_SENSOR_ROTATION_PITCH_270", 25 }, \
    { "MAV_SENSOR_ROTATION_PITCH_180_YAW_90", 26 }, \
    { "MAV_SENSOR_ROTATION_PITCH_180_YAW_270", 27 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_PITCH_90", 28 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180_PITCH_90", 29 }, \
    { "MAV_SENSOR_ROTATION_ROLL_270_PITCH_90", 30 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_PITCH_180", 31 }, \
    { "MAV_SENSOR_ROTATION_ROLL_270_PITCH_180", 32 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_PITCH_270", 33 }, \
    { "MAV_SENSOR_ROTATION_ROLL_180_PITCH_270", 34 }, \
    { "MAV_SENSOR_ROTATION_ROLL_270_PITCH_270", 35 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_PITCH_180_YAW_90", 36 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_YAW_270", 37 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_PITCH_68_YAW_293", 38 }, \
    { "MAV_SENSOR_ROTATION_PITCH_315", 39 }, \
    { "MAV_SENSOR_ROTATION_ROLL_90_PITCH_315", 40 }, \
    { "MAV_SENSOR_ROTATION_CUSTOM", 100 }, \
    \
    { "MAV_PROTOCOL_CAPABILITY_MISSION_FLOAT", 1 }, \
    { "MAV_PROTOCOL_CAPABILITY_PARAM_FLOAT", 2 }, \
    { "MAV_PROTOCOL_CAPABILITY_MISSION_INT", 4 }, \
    { "MAV_PROTOCOL_CAPABILITY_COMMAND_INT", 8 }, \
    { "MAV_PROTOCOL_CAPABILITY_PARAM_UNION", 16 }, \
    { "MAV_PROTOCOL_CAPABILITY_FTP", 32 }, \
    { "MAV_PROTOCOL_CAPABILITY_SET_ATTITUDE_TARGET", 64 }, \
    { "MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_LOCAL_NED", 128 }, \
    { "MAV_PROTOCOL_CAPABILITY_SET_POSITION_TARGET_GLOBAL_INT", 256 }, \
    { "MAV_PROTOCOL_CAPABILITY_TERRAIN", 512 }, \
    { "MAV_PROTOCOL_CAPABILITY_SET_ACTUATOR_TARGET", 1024 }, \
    { "MAV_PROTOCOL_CAPABILITY_FLIGHT_TERMINATION", 2048 }, \
    { "MAV_PROTOCOL_CAPABILITY_COMPASS_CALIBRATION", 4096 }, \
    { "MAV_PROTOCOL_CAPABILITY_MAVLINK2", 8192 }, \
    { "MAV_PROTOCOL_CAPABILITY_MISSION_FENCE", 16384 }, \
    { "MAV_PROTOCOL_CAPABILITY_MISSION_RALLY", 32768 }, \
    { "MAV_PROTOCOL_CAPABILITY_FLIGHT_INFORMATION", 65536 }, \
    \
    { "MAV_MISSION_TYPE_MISSION", 0 }, \
    { "MAV_MISSION_TYPE_FENCE", 1 }, \
    { "MAV_MISSION_TYPE_RALLY", 2 }, \
    { "MAV_MISSION_TYPE_ALL", 255 }, \
    \
    { "MAV_ESTIMATOR_TYPE_UNKNOWN", 0 }, \
    { "MAV_ESTIMATOR_TYPE_NAIVE", 1 }, \
    { "MAV_ESTIMATOR_TYPE_VISION", 2 }, \
    { "MAV_ESTIMATOR_TYPE_VIO", 3 }, \
    { "MAV_ESTIMATOR_TYPE_GPS", 4 }, \
    { "MAV_ESTIMATOR_TYPE_GPS_INS", 5 }, \
    { "MAV_ESTIMATOR_TYPE_MOCAP", 6 }, \
    { "MAV_ESTIMATOR_TYPE_LIDAR", 7 }, \
    { "MAV_ESTIMATOR_TYPE_AUTOPILOT", 8 }, \
    \
    { "MAV_BATTERY_TYPE_UNKNOWN", 0 }, \
    { "MAV_BATTERY_TYPE_LIPO", 1 }, \
    { "MAV_BATTERY_TYPE_LIFE", 2 }, \
    { "MAV_BATTERY_TYPE_LION", 3 }, \
    { "MAV_BATTERY_TYPE_NIMH", 4 }, \
    \
    { "MAV_BATTERY_FUNCTION_UNKNOWN", 0 }, \
    { "MAV_BATTERY_FUNCTION_ALL", 1 }, \
    { "MAV_BATTERY_FUNCTION_PROPULSION", 2 }, \
    { "MAV_BATTERY_FUNCTION_AVIONICS", 3 }, \
    { "MAV_BATTERY_TYPE_PAYLOAD", 4 }, \
    \
    { "MAV_BATTERY_CHARGE_STATE_UNDEFINED", 0 }, \
    { "MAV_BATTERY_CHARGE_STATE_OK", 1 }, \
    { "MAV_BATTERY_CHARGE_STATE_LOW", 2 }, \
    { "MAV_BATTERY_CHARGE_STATE_CRITICAL", 3 }, \
    { "MAV_BATTERY_CHARGE_STATE_EMERGENCY", 4 }, \
    { "MAV_BATTERY_CHARGE_STATE_FAILED", 5 }, \
    { "MAV_BATTERY_CHARGE_STATE_UNHEALTHY", 6 }, \
    { "MAV_BATTERY_CHARGE_STATE_CHARGING", 7 }, \
    \
    { "MAV_SMART_BATTERY_FAULT_DEEP_DISCHARGE", 1 }, \
    { "MAV_SMART_BATTERY_FAULT_SPIKES", 2 }, \
    { "MAV_SMART_BATTERY_FAULT_SINGLE_CELL_FAIL", 4 }, \
    { "MAV_SMART_BATTERY_FAULT_OVER_CURRENT", 8 }, \
    { "MAV_SMART_BATTERY_FAULT_OVER_TEMPERATURE", 16 }, \
    { "MAV_SMART_BATTERY_FAULT_UNDER_TEMPERATURE", 32 }, \
    \
    { "MAV_VTOL_STATE_UNDEFINED", 0 }, \
    { "MAV_VTOL_STATE_TRANSITION_TO_FW", 1 }, \
    { "MAV_VTOL_STATE_TRANSITION_TO_MC", 2 }, \
    { "MAV_VTOL_STATE_MC", 3 }, \
    { "MAV_VTOL_STATE_FW", 4 }, \
    \
    { "MAV_LANDED_STATE_UNDEFINED", 0 }, \
    { "MAV_LANDED_STATE_ON_GROUND", 1 }, \
    { "MAV_LANDED_STATE_IN_AIR", 2 }, \
    { "MAV_LANDED_STATE_TAKEOFF", 3 }, \
    { "MAV_LANDED_STATE_LANDING", 4 }, \
    \
    { "ADSB_ALTITUDE_TYPE_PRESSURE_QNH", 0 }, \
    { "ADSB_ALTITUDE_TYPE_GEOMETRIC", 1 }, \
    \
    { "ADSB_EMITTER_TYPE_NO_INFO", 0 }, \
    { "ADSB_EMITTER_TYPE_LIGHT", 1 }, \
    { "ADSB_EMITTER_TYPE_SMALL", 2 }, \
    { "ADSB_EMITTER_TYPE_LARGE", 3 }, \
    { "ADSB_EMITTER_TYPE_HIGH_VORTEX_LARGE", 4 }, \
    { "ADSB_EMITTER_TYPE_HEAVY", 5 }, \
    { "ADSB_EMITTER_TYPE_HIGHLY_MANUV", 6 }, \
    { "ADSB_EMITTER_TYPE_ROTOCRAFT", 7 }, \
    { "ADSB_EMITTER_TYPE_UNASSIGNED", 8 }, \
    { "ADSB_EMITTER_TYPE_GLIDER", 9 }, \
    { "ADSB_EMITTER_TYPE_LIGHTER_AIR", 10 }, \
    { "ADSB_EMITTER_TYPE_PARACHUTE", 11 }, \
    { "ADSB_EMITTER_TYPE_ULTRA_LIGHT", 12 }, \
    { "ADSB_EMITTER_TYPE_UNASSIGNED2", 13 }, \
    { "ADSB_EMITTER_TYPE_UAV", 14 }, \
    { "ADSB_EMITTER_TYPE_SPACE", 15 }, \
    { "ADSB_EMITTER_TYPE_UNASSGINED3", 16 }, \
    { "ADSB_EMITTER_TYPE_EMERGENCY_SURFACE", 17 }, \
    { "ADSB_EMITTER_TYPE_SERVICE_SURFACE", 18 }, \
    { "ADSB_EMITTER_TYPE_POINT_OBSTACLE", 19 }, \
    \
    { "ADSB_FLAGS_VALID_COORDS", 1 }, \
    { "ADSB_FLAGS_VALID_ALTITUDE", 2 }, \
    { "ADSB_FLAGS_VALID_HEADING", 4 }, \
    { "ADSB_FLAGS_VALID_VELOCITY", 8 }, \
    { "ADSB_FLAGS_VALID_CALLSIGN", 16 }, \
    { "ADSB_FLAGS_VALID_SQUAWK", 32 }, \
    { "ADSB_FLAGS_SIMULATED", 64 }, \
    { "ADSB_FLAGS_VERTICAL_VELOCITY_VALID", 128 }, \
    { "ADSB_FLAGS_BARO_VALID", 256 }, \
    { "ADSB_FLAGS_SOURCE_UAT", 32768 }, \
    \
    { "MAV_DO_REPOSITION_FLAGS_CHANGE_MODE", 1 }, \
    \
    { "ESTIMATOR_ATTITUDE", 1 }, \
    { "ESTIMATOR_VELOCITY_HORIZ", 2 }, \
    { "ESTIMATOR_VELOCITY_VERT", 4 }, \
    { "ESTIMATOR_POS_HORIZ_REL", 8 }, \
    { "ESTIMATOR_POS_HORIZ_ABS", 16 }, \
    { "ESTIMATOR_POS_VERT_ABS", 32 }, \
    { "ESTIMATOR_POS_VERT_AGL", 64 }, \
    { "ESTIMATOR_CONST_POS_MODE", 128 }, \
    { "ESTIMATOR_PRED_POS_HORIZ_REL", 256 }, \
    { "ESTIMATOR_PRED_POS_HORIZ_ABS", 512 }, \
    { "ESTIMATOR_GPS_GLITCH", 1024 }, \
    { "ESTIMATOR_ACCEL_ERROR", 2048 }, \
    \
    { "MOTOR_TEST_ORDER_DEFAULT", 0 }, \
    { "MOTOR_TEST_ORDER_SEQUENCE", 1 }, \
    { "MOTOR_TEST_ORDER_BOARD", 2 }, \
    \
    { "MOTOR_TEST_THROTTLE_PERCENT", 0 }, \
    { "MOTOR_TEST_THROTTLE_PWM", 1 }, \
    { "MOTOR_TEST_THROTTLE_PILOT", 2 }, \
    { "MOTOR_TEST_COMPASS_CAL", 3 }, \
    \
    { "GPS_INPUT_IGNORE_FLAG_ALT", 1 }, \
    { "GPS_INPUT_IGNORE_FLAG_HDOP", 2 }, \
    { "GPS_INPUT_IGNORE_FLAG_VDOP", 4 }, \
    { "GPS_INPUT_IGNORE_FLAG_VEL_HORIZ", 8 }, \
    { "GPS_INPUT_IGNORE_FLAG_VEL_VERT", 16 }, \
    { "GPS_INPUT_IGNORE_FLAG_SPEED_ACCURACY", 32 }, \
    { "GPS_INPUT_IGNORE_FLAG_HORIZONTAL_ACCURACY", 64 }, \
    { "GPS_INPUT_IGNORE_FLAG_VERTICAL_ACCURACY", 128 }, \
    \
    { "MAV_COLLISION_ACTION_NONE", 0 }, \
    { "MAV_COLLISION_ACTION_REPORT", 1 }, \
    { "MAV_COLLISION_ACTION_ASCEND_OR_DESCEND", 2 }, \
    { "MAV_COLLISION_ACTION_MOVE_HORIZONTALLY", 3 }, \
    { "MAV_COLLISION_ACTION_MOVE_PERPENDICULAR", 4 }, \
    { "MAV_COLLISION_ACTION_RTL", 5 }, \
    { "MAV_COLLISION_ACTION_HOVER", 6 }, \
    \
    { "MAV_COLLISION_THREAT_LEVEL_NONE", 0 }, \
    { "MAV_COLLISION_THREAT_LEVEL_LOW", 1 }, \
    { "MAV_COLLISION_THREAT_LEVEL_HIGH", 2 }, \
    \
    { "MAV_COLLISION_SRC_ADSB", 0 }, \
    { "MAV_COLLISION_SRC_MAVLINK_GPS_GLOBAL_INT", 1 }, \
    \
    { "GPS_FIX_TYPE_NO_GPS", 0 }, \
    { "GPS_FIX_TYPE_NO_FIX", 1 }, \
    { "GPS_FIX_TYPE_2D_FIX", 2 }, \
    { "GPS_FIX_TYPE_3D_FIX", 3 }, \
    { "GPS_FIX_TYPE_DGPS", 4 }, \
    { "GPS_FIX_TYPE_RTK_FLOAT", 5 }, \
    { "GPS_FIX_TYPE_RTK_FIXED", 6 }, \
    { "GPS_FIX_TYPE_STATIC", 7 }, \
    { "GPS_FIX_TYPE_PPP", 8 }, \
    \
    { "RTK_BASELINE_COORDINATE_SYSTEM_ECEF", 0 }, \
    { "RTK_BASELINE_COORDINATE_SYSTEM_NED", 1 }, \
    \
    { "LANDING_TARGET_TYPE_LIGHT_BEACON", 0 }, \
    { "LANDING_TARGET_TYPE_RADIO_BEACON", 1 }, \
    { "LANDING_TARGET_TYPE_VISION_FIDUCIAL", 2 }, \
    { "LANDING_TARGET_TYPE_VISION_OTHER", 3 }, \
    \
    { "VTOL_TRANSITION_HEADING_VEHICLE_DEFAULT", 0 }, \
    { "VTOL_TRANSITION_HEADING_NEXT_WAYPOINT", 1 }, \
    { "VTOL_TRANSITION_HEADING_TAKEOFF", 2 }, \
    { "VTOL_TRANSITION_HEADING_SPECIFIED", 3 }, \
    { "VTOL_TRANSITION_HEADING_ANY", 4 }, \
    \
    { "CAMERA_CAP_FLAGS_CAPTURE_VIDEO", 1 }, \
    { "CAMERA_CAP_FLAGS_CAPTURE_IMAGE", 2 }, \
    { "CAMERA_CAP_FLAGS_HAS_MODES", 4 }, \
    { "CAMERA_CAP_FLAGS_CAN_CAPTURE_IMAGE_IN_VIDEO_MODE", 8 }, \
    { "CAMERA_CAP_FLAGS_CAN_CAPTURE_VIDEO_IN_IMAGE_MODE", 16 }, \
    { "CAMERA_CAP_FLAGS_HAS_IMAGE_SURVEY_MODE", 32 }, \
    { "CAMERA_CAP_FLAGS_HAS_BASIC_ZOOM", 64 }, \
    { "CAMERA_CAP_FLAGS_HAS_BASIC_FOCUS", 128 }, \
    { "CAMERA_CAP_FLAGS_HAS_VIDEO_STREAM", 256 }, \
    \
    { "VIDEO_STREAM_STATUS_FLAGS_RUNNING", 1 }, \
    { "VIDEO_STREAM_STATUS_FLAGS_THERMAL", 2 }, \
    \
    { "VIDEO_STREAM_TYPE_RTSP", 0 }, \
    { "VIDEO_STREAM_TYPE_RTPUDP", 1 }, \
    { "VIDEO_STREAM_TYPE_TCP_MPEG", 2 }, \
    { "VIDEO_STREAM_TYPE_MPEG_TS_H264", 3 }, \
    \
    { "ZOOM_TYPE_STEP", 0 }, \
    { "ZOOM_TYPE_CONTINUOUS", 1 }, \
    { "ZOOM_TYPE_RANGE", 2 }, \
    { "ZOOM_TYPE_FOCAL_LENGTH", 3 }, \
    \
    { "FOCUS_TYPE_STEP", 0 }, \
    { "FOCUS_TYPE_CONTINUOUS", 1 }, \
    { "FOCUS_TYPE_RANGE", 2 }, \
    { "FOCUS_TYPE_METERS", 3 }, \
    \
    { "PARAM_ACK_ACCEPTED", 0 }, \
    { "PARAM_ACK_VALUE_UNSUPPORTED", 1 }, \
    { "PARAM_ACK_FAILED", 2 }, \
    { "PARAM_ACK_IN_PROGRESS", 3 }, \
    \
    { "CAMERA_MODE_IMAGE", 0 }, \
    { "CAMERA_MODE_VIDEO", 1 }, \
    { "CAMERA_MODE_IMAGE_SURVEY", 2 }, \
    \
    { "MAV_ARM_AUTH_DENIED_REASON_GENERIC", 0 }, \
    { "MAV_ARM_AUTH_DENIED_REASON_NONE", 1 }, \
    { "MAV_ARM_AUTH_DENIED_REASON_INVALID_WAYPOINT", 2 }, \
    { "MAV_ARM_AUTH_DENIED_REASON_TIMEOUT", 3 }, \
    { "MAV_ARM_AUTH_DENIED_REASON_AIRSPACE_IN_USE", 4 }, \
    { "MAV_ARM_AUTH_DENIED_REASON_BAD_WEATHER", 5 }, \
    \
    { "RC_TYPE_SPEKTRUM_DSM2", 0 }, \
    { "RC_TYPE_SPEKTRUM_DSMX", 1 }, \
    \
    { "POSITION_TARGET_TYPEMASK_X_IGNORE", 1 }, \
    { "POSITION_TARGET_TYPEMASK_Y_IGNORE", 2 }, \
    { "POSITION_TARGET_TYPEMASK_Z_IGNORE", 4 }, \
    { "POSITION_TARGET_TYPEMASK_VX_IGNORE", 8 }, \
    { "POSITION_TARGET_TYPEMASK_VY_IGNORE", 16 }, \
    { "POSITION_TARGET_TYPEMASK_VZ_IGNORE", 32 }, \
    { "POSITION_TARGET_TYPEMASK_AX_IGNORE", 64 }, \
    { "POSITION_TARGET_TYPEMASK_AY_IGNORE", 128 }, \
    { "POSITION_TARGET_TYPEMASK_AZ_IGNORE", 256 }, \
    { "POSITION_TARGET_TYPEMASK_FORCE_SET", 512 }, \
    { "POSITION_TARGET_TYPEMASK_YAW_IGNORE", 1024 }, \
    { "POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE", 2048 }, \
    \
    { "UTM_FLIGHT_STATE_UNKNOWN", 1 }, \
    { "UTM_FLIGHT_STATE_GROUND", 2 }, \
    { "UTM_FLIGHT_STATE_AIRBORNE", 3 }, \
    { "UTM_FLIGHT_STATE_EMERGENCY", 16 }, \
    { "UTM_FLIGHT_STATE_NOCTRL", 32 }, \
    \
    { "UTM_DATA_AVAIL_FLAGS_TIME_VALID", 1 }, \
    { "UTM_DATA_AVAIL_FLAGS_UAS_ID_AVAILABLE", 2 }, \
    { "UTM_DATA_AVAIL_FLAGS_POSITION_AVAILABLE", 4 }, \
    { "UTM_DATA_AVAIL_FLAGS_ALTITUDE_AVAILABLE", 8 }, \
    { "UTM_DATA_AVAIL_FLAGS_RELATIVE_ALTITUDE_AVAILABLE", 16 }, \
    { "UTM_DATA_AVAIL_FLAGS_HORIZONTAL_VELO_AVAILABLE", 32 }, \
    { "UTM_DATA_AVAIL_FLAGS_VERTICAL_VELO_AVAILABLE", 64 }, \
    { "UTM_DATA_AVAIL_FLAGS_NEXT_WAYPOINT_AVAILABLE", 128 }, \
    \
    { "CELLULAR_NETWORK_RADIO_TYPE_NONE", 0 }, \
    { "CELLULAR_NETWORK_RADIO_TYPE_GSM", 1 }, \
    { "CELLULAR_NETWORK_RADIO_TYPE_CDMA", 2 }, \
    { "CELLULAR_NETWORK_RADIO_TYPE_WCDMA", 3 }, \
    { "CELLULAR_NETWORK_RADIO_TYPE_LTE", 4 }, \
    \
    { "CELLULAR_NETWORK_STATUS_FLAG_ROAMING", 1 }, \
    \
    { "PRECISION_LAND_MODE_DISABLED", 0 }, \
    { "PRECISION_LAND_MODE_OPPORTUNISTIC", 1 }, \
    { "PRECISION_LAND_MODE_REQUIRED", 2 }, \
    \
    { "PARACHUTE_DISABLE", 0 }, \
    { "PARACHUTE_ENABLE", 1 }, \
    { "PARACHUTE_RELEASE", 2 }, \
    \
    { "MAV_TUNNEL_PAYLOAD_TYPE_UNKNOWN", 0 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED0", 200 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED1", 201 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED2", 202 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED3", 203 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED4", 204 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED5", 205 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED6", 206 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED7", 207 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED8", 208 }, \
    { "MAV_TUNNEL_PAYLOAD_TYPE_STORM32_RESERVED9", 209 }, \
    \
    { "TUNE_FORMAT_QBASIC1_1", 1 }, \
    { "TUNE_FORMAT_MML_MODERN", 2 }, \
    \
    { "COMPONENT_CAP_FLAGS_PARAM", 1 }, \
    { "COMPONENT_CAP_FLAGS_PARAM_EXT", 2 }, \
    \
    { "UNDER_WAY", 0 }, \


const luaL_Reg mavlinkLib[] = {
#if defined(MAVLINK_TELEM)
    { "getVersion", luaMavlinkGetVersion },
    { "getChannelStatus", luaMavlinkGetChannelStatus },
//    { "available", luaMavlinkMsgAvailable },
//    { "probeHeader", luaMavlinkMsgProbeHeader },
//    { "popAndDiscard", luaMavlinkMsgPopAndDiscard },
//    { "clear", luaMavlinkMsgClear },
//    { "enable", luaMavlinkMsgEnable },

//    MAVLINK_LIB_FUNCTIONS
#endif
    { nullptr, nullptr }  /* sentinel */
};

const luaR_value_entry mavlinkConstants[] = {
#if defined(MAVLINK_TELEM)
    MAVLINK_LIB_CONSTANTS
#endif
  { nullptr, 0 }  /* sentinel */
};

#endif
