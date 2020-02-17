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


// -- GIMBAL --

static int luaMavsdkGimbalIsReceiving(lua_State *L)
{
    bool flag = (mavlinkTelem.gimbal.is_receiving > 0);
    lua_pushboolean(L, flag);
    return 1;
}

static int luaMavsdkGimbalGetInfo(lua_State *L)
{
	lua_newtable(L);
	lua_pushtableinteger(L, "compid", mavlinkTelem.gimbal.compid);
	return 1;
}

static int luaMavsdkGimbalGetStatus(lua_State *L)
{
	lua_newtable(L);
	lua_pushtablenumber(L, "system_status", mavlinkTelem.gimbal.system_status);
	lua_pushtablenumber(L, "custom_mode", mavlinkTelem.gimbal.custom_mode);
	lua_pushtableboolean(L, "is_armed", mavlinkTelem.gimbal.is_armed);
	lua_pushtableboolean(L, "prearm_ok", mavlinkTelem.gimbal.prearm_ok);
	return 1;
}

static int luaMavsdkGimbalGetAttRollDeg(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gimbalAtt.roll*(180.0/3.141592654));
	return 1;
}

static int luaMavsdkGimbalGetAttPitchDeg(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gimbalAtt.pitch*(180.0/3.141592654));
	return 1;
}

static int luaMavsdkGimbalGetAttYawDeg(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gimbalAtt.yaw_relative*(180.0/3.141592654));
	return 1;
}

static int luaMavsdkGimbalSetNeutralMode(lua_State *L)
{
	mavlinkTelem.setGimbalTargetingMode(MAV_MOUNT_MODE_NEUTRAL);
	return 0;
}

static int luaMavsdkGimbalSetMavlinkTargetingMode(lua_State *L)
{
	mavlinkTelem.setGimbalTargetingMode(MAV_MOUNT_MODE_MAVLINK_TARGETING);
	return 0;
}

static int luaMavsdkGimbalSetRcTargetingMode(lua_State *L)
{
	mavlinkTelem.setGimbalTargetingMode(MAV_MOUNT_MODE_RC_TARGETING);
	return 0;
}

static int luaMavsdkGimbalSetGpsPointMode(lua_State *L)
{
	mavlinkTelem.setGimbalTargetingMode(MAV_MOUNT_MODE_GPS_POINT);
	return 0;
}

static int luaMavsdkGimbalSetSysIdTargetingMode(lua_State *L)
{
	mavlinkTelem.setGimbalTargetingMode(MAV_MOUNT_MODE_SYSID_TARGET);
	return 0;
}

static int luaMavsdkGimbalSetPitchYawDeg(lua_State *L)
{
	float pitch = luaL_checknumber(L, 1);
	float yaw = luaL_checknumber(L, 2);
	mavlinkTelem.setGimbalPitchYawDeg(pitch, yaw);
	return 0;
}


// -- CAMERA --

static int luaMavsdkCameraIsReceiving(lua_State *L)
{
    bool flag = (mavlinkTelem.camera.is_receiving > 0);
    lua_pushboolean(L, flag);
    return 1;
}

static int luaMavsdkCameraIsInitialized(lua_State *L)
{
    bool flag = (mavlinkTelem.camera.is_receiving > 0) & mavlinkTelem.cameraStatus.initialized;
    lua_pushboolean(L, flag);
    return 1;
}

static int luaMavsdkCameraGetInfo(lua_State *L)
{
	lua_newtable(L);
	lua_pushtableinteger(L, "compid", mavlinkTelem.camera.compid);
	lua_pushtableinteger(L, "flags", mavlinkTelem.cameraInfo.flags);
	lua_pushtableboolean(L, "has_video", mavlinkTelem.cameraInfo.has_video);
	lua_pushtableboolean(L, "has_photo", mavlinkTelem.cameraInfo.has_photo);
	lua_pushtableboolean(L, "has_modes", mavlinkTelem.cameraInfo.has_modes);
	if (!isnan(mavlinkTelem.cameraInfo.total_capacity)) {
		lua_pushtablenumber(L, "total_capacity", mavlinkTelem.cameraInfo.total_capacity);
	} else {
		lua_pushtablenil(L, "total_capacity");
	}
	lua_pushtablestring(L, "vendor_name", mavlinkTelem.cameraInfo.vendor_name);
	lua_pushtablestring(L, "model_name", mavlinkTelem.cameraInfo.model_name);
	return 1;
}

static int luaMavsdkCameraGetStatus(lua_State *L)
{
	lua_newtable(L);
	lua_pushtableinteger(L, "system_status", mavlinkTelem.camera.system_status);
	lua_pushtableboolean(L, "initialized", mavlinkTelem.cameraStatus.initialized);
	lua_pushtableinteger(L, "mode", mavlinkTelem.cameraStatus.mode);
	lua_pushtableboolean(L, "video_on", mavlinkTelem.cameraStatus.video_on);
	lua_pushtableboolean(L, "photo_on", mavlinkTelem.cameraStatus.photo_on);
	if (!isnan(mavlinkTelem.cameraStatus.available_capacity)) {
		lua_pushtablenumber(L, "available_capacity", mavlinkTelem.cameraStatus.available_capacity);
	} else {
		lua_pushtablenil(L, "available_capacity");
	}
	if (!isnan(mavlinkTelem.cameraStatus.battery_voltage)) {
		lua_pushtablenumber(L, "battery_voltage", mavlinkTelem.cameraStatus.battery_voltage);
	} else {
		lua_pushtablenil(L, "battery_voltage");
	}
	if (mavlinkTelem.cameraStatus.battery_remainingpct >= 0) {
		lua_pushtableinteger(L, "battery_remainingpct", mavlinkTelem.cameraStatus.battery_remainingpct);
	} else {
		lua_pushtablenil(L, "battery_remainingpct");
	}
	return 1;
}

static int luaMavsdkCameraSetVideoMode(lua_State *L)
{
	mavlinkTelem.setCameraSetVideoMode();
	return 0;
}

static int luaMavsdkCameraSetPhotoMode(lua_State *L)
{
	mavlinkTelem.setCameraSetPhotoMode();
	return 0;
}

static int luaMavsdkCameraStartVideo(lua_State *L)
{
	mavlinkTelem.setCameraStartVideo();
	return 0;
}

static int luaMavsdkCameraStopVideo(lua_State *L)
{
	mavlinkTelem.setCameraStopVideo();
	return 0;
}

static int luaMavsdkCameraTakePhoto(lua_State *L)
{
	mavlinkTelem.setCameraTakePhoto();
	return 0;
}


// -- MAVSDK GENERAL --

static int luaMavsdkIsReceiving(lua_State *L)
{
    bool flag = mavlinkTelem.isReceiving();
    lua_pushboolean(L, flag);
    return 1;
}

static int luaMavsdkGetAutopilotType(lua_State *L)
{
    int nbr = mavlinkTelem.autopilottype;
    lua_pushnumber(L, nbr);
    return 1;
}

static int luaMavsdkGetVehicleType(lua_State *L)
{
    int nbr = mavlinkTelem.vehicletype;
	lua_pushnumber(L, nbr);
	return 1;
}

static int luaMavsdkGetFlightMode(lua_State *L)
{
    int nbr = mavlinkTelem.flightmode;
	lua_pushnumber(L, nbr);
	return 1;
}

typedef enum MAVSDK_VEHICLECLASS {
   MAVSDK_VEHICLECLASS_GENERIC = 0,
   MAVSDK_VEHICLECLASS_PLANE,
   MAVSDK_VEHICLECLASS_COPTER,
   MAVSDK_VEHICLECLASS_ROVER,
   MAVSDK_VEHICLECLASS_BOAT,
   MAVSDK_VEHICLECLASS_SUB,
   MAVSDK_VEHICLECLASS_ENUM_END
} MAVSDK_VEHICLECLASS;

static int luaMavsdkGetVehicleClass(lua_State *L)
{
int nbr;

	switch (mavlinkTelem.vehicletype) {
	case MAV_TYPE_FIXED_WING:
	case MAV_TYPE_FLAPPING_WING:
	case MAV_TYPE_VTOL_DUOROTOR:
	case MAV_TYPE_VTOL_QUADROTOR:
	case MAV_TYPE_VTOL_TILTROTOR:
	case MAV_TYPE_VTOL_RESERVED2:
	case MAV_TYPE_VTOL_RESERVED3:
	case MAV_TYPE_VTOL_RESERVED4:
	case MAV_TYPE_VTOL_RESERVED5:
	case MAV_TYPE_PARAFOIL:
		nbr = MAVSDK_VEHICLECLASS_PLANE;
		break;
	case MAV_TYPE_QUADROTOR:
	case MAV_TYPE_COAXIAL:
	case MAV_TYPE_HELICOPTER:
	case MAV_TYPE_HEXAROTOR:
	case MAV_TYPE_OCTOROTOR:
	case MAV_TYPE_TRICOPTER:
	case MAV_TYPE_DODECAROTOR:
		nbr = MAVSDK_VEHICLECLASS_COPTER;
		break;
	case MAV_TYPE_GROUND_ROVER:
		nbr = MAVSDK_VEHICLECLASS_ROVER;
		break;
	case MAV_TYPE_SURFACE_BOAT:
		nbr = MAVSDK_VEHICLECLASS_BOAT;
		break;
	case MAV_TYPE_SUBMARINE:
		nbr = MAVSDK_VEHICLECLASS_SUB;
		break;
	default:
		nbr = MAVSDK_VEHICLECLASS_GENERIC;
	}
	lua_pushnumber(L, nbr);
	return 1;
}

static int luaMavsdkGetSystemStatus(lua_State *L)
{
    int nbr = mavlinkTelem.autopilot.system_status;
	lua_pushnumber(L, nbr);
	return 1;
}

static int luaMavsdkIsArmed(lua_State *L)
{
    bool flag = mavlinkTelem.autopilot.is_armed;
    lua_pushboolean(L, flag);
    return 1;
}


// -- MAVSDK RADIO  --

static int luaMavsdkGetRadioRssi(lua_State *L)
{
	lua_pushinteger(L, mavlinkTelem.radio.rssi);
	return 1;
}

static int luaMavsdkGetRadioRemoteRssi(lua_State *L)
{
	lua_pushinteger(L, mavlinkTelem.radio.remrssi);
	return 1;
}

static int luaMavsdkGetRadioNoise(lua_State *L)
{
	lua_pushinteger(L, mavlinkTelem.radio.noise);
	return 1;
}

static int luaMavsdkGetRadioRemoteNoise(lua_State *L)
{
	lua_pushinteger(L, mavlinkTelem.radio.remnoise);
	return 1;
}


// -- MAVSDK ATTITUDE --

static int luaMavsdkGetAttRollDeg(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.att.roll*(180.0/3.141592654));
	return 1;
}

static int luaMavsdkGetAttPitchDeg(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.att.pitch*(180.0/3.141592654));
	return 1;
}

static int luaMavsdkGetAttYawDeg(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.att.yaw*(180.0/3.141592654));
	return 1;
}


// -- MAVSDK GPS --

static int luaMavsdkGetGpsStatus(lua_State *L)
{
/*	lua_createtable(L, 0, 4);
	lua_pushtablenumber(L, "fix", mavlinkTelem.gps_fix);
	lua_pushtablenumber(L, "hdop", mavlinkTelem.gps_hdop * 0.01);
	lua_pushtablenumber(L, "vdop", mavlinkTelem.gps_vdop * 0.01);
	lua_pushtablenumber(L, "sat", mavlinkTelem.gps_sat);
*/
	lua_newtable(L);
	lua_pushtablenumber(L, "fix", mavlinkTelem.gps.fix);
	lua_pushtablenumber(L, "hdop", mavlinkTelem.gps.hdop * 0.01);
	lua_pushtablenumber(L, "vdop", mavlinkTelem.gps.vdop * 0.01);
	lua_pushtablenumber(L, "sat", mavlinkTelem.gps.sat);
	return 1;
}

static int luaMavsdkGetGpsFix(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gps.fix);
	return 1;
}

static int luaMavsdkGetGpsHDop(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gps.hdop * 0.01);
	return 1;
}

static int luaMavsdkGetGpsVDop(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gps.vdop * 0.01);
	return 1;
}

static int luaMavsdkGetGpsSat(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gps.sat);
	return 1;
}

static int luaMavsdkGetGpsLatLonDeg(lua_State *L)
{
	lua_newtable(L);
    lua_pushtablenumber(L, "lat", mavlinkTelem.gps.lat * 1.0E-7);
    lua_pushtablenumber(L, "lon", mavlinkTelem.gps.lon * 1.0E-7);
	return 1;
}

static int luaMavsdkGetGpsAltitudeAmsl(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gps.alt * 0.001);
	return 1;
}

static int luaMavsdkGetGpsSpeed(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gps.vel * 0.01);
	return 1;
}

static int luaMavsdkGetGpsCourseOverGround(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gps.cog * 0.01);
	return 1;
}


// -- MAVSDK POSITION --

static int luaMavsdkGetPositionAltitudeRelative(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.gposition.relative_alt * 0.001);
	return 1;
}


// -- MAVSDK VFR --

static int luaMavsdkGetVfrAirSpeed(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.vfr.airspd);
	return 1;
}

static int luaMavsdkGetVfrGroundSpeed(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.vfr.groundspd);
	return 1;
}

static int luaMavsdkGetVfrAltitudeMsl(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.vfr.alt);
	return 1;
}

static int luaMavsdkGetVfrClimbRate(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.vfr.climbrate);
	return 1;
}

static int luaMavsdkGetVfrHeading(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.vfr.heading);
	return 1;
}

static int luaMavsdkGetVfrThrottle(lua_State *L)
{
	lua_pushinteger(L, mavlinkTelem.vfr.thro);
	return 1;
}


// -- MAVSDK BATTERY --

static int luaMavsdkGetBat1ChargeConsumed(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat1.charge_consumed);
	return 1;
}

static int luaMavsdkGetBat1EnergyConsumed(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat1.energy_consumed * 100.0);
	return 1;
}

static int luaMavsdkGetBat1Temperature(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat1.temperature * 0.01);
	return 1;
}

static int luaMavsdkGetBat1Voltage(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat1.voltage * 0.001);
	return 1;
}

static int luaMavsdkGetBat1Current(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat1.current * 0.01);
	return 1;
}

static int luaMavsdkGetBat1Remaining(lua_State *L)
{
    lua_pushinteger(L, mavlinkTelem.bat1.remainingpct);
	return 1;
}

static int luaMavsdkGetBat1CellCount(lua_State *L)
{
    lua_pushinteger(L, mavlinkTelem.bat1.cellcount);
	return 1;
}


static int luaMavsdkGetBat2ChargeConsumed(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat2.charge_consumed);
	return 1;
}

static int luaMavsdkGetBat2EnergyConsumed(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat2.energy_consumed * 100.0);
	return 1;
}

static int luaMavsdkGetBat2Temperature(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat2.temperature * 0.01);
	return 1;
}

static int luaMavsdkGetBat2Voltage(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat2.voltage * 0.001);
	return 1;
}

static int luaMavsdkGetBat2Current(lua_State *L)
{
    lua_pushnumber(L, mavlinkTelem.bat2.current * 0.01);
	return 1;
}

static int luaMavsdkGetBat2Remaining(lua_State *L)
{
    lua_pushinteger(L, mavlinkTelem.bat2.remainingpct);
	return 1;
}

static int luaMavsdkGetBat2CellCount(lua_State *L)
{
    lua_pushinteger(L, mavlinkTelem.bat2.cellcount);
	return 1;
}


static int luaMavsdkGetBatCount(lua_State *L)
{
	uint16_t cnt = 0, mask = mavlinkTelem.bat_instancemask;
	for(uint8_t i = 0; i < 8; i++) { if (mask & 0x01) cnt++; mask >>= 1; }
    lua_pushinteger(L, cnt);
	return 1;
}



const luaL_Reg mavsdkLib[] = {
#if defined(MAVLINK_TELEM)
  { "isReceiving", luaMavsdkIsReceiving },			 // bool
  { "getAutopilotType", luaMavsdkGetAutopilotType }, // MAV_AUTOPILOT_xxx
  { "getVehicleType", luaMavsdkGetVehicleType },	 // MAV_TYPE_xxx
  { "getFlightMode", luaMavsdkGetFlightMode },		 // MAVSDK_VEHICLECLASS_xxx
  { "getVehicleClass", luaMavsdkGetVehicleClass },	 // no enum, just raw number
  { "getSystemStatus", luaMavsdkGetSystemStatus },	 // bool
  { "isArmed", luaMavsdkIsArmed },

  { "gimbalIsReceiving", luaMavsdkGimbalIsReceiving },
  { "gimbalGetInfo", luaMavsdkGimbalGetInfo },
  { "gimbalGetStatus", luaMavsdkGimbalGetStatus },
  { "gimbalGetAttRollDeg", luaMavsdkGimbalGetAttRollDeg },
  { "gimbalGetAttPitchDeg", luaMavsdkGimbalGetAttPitchDeg },
  { "gimbalGetAttYawDeg", luaMavsdkGimbalGetAttYawDeg },
  { "gimbalSetNeutralMode", luaMavsdkGimbalSetNeutralMode },
  { "gimbalSetMavlinkTargetingMode", luaMavsdkGimbalSetMavlinkTargetingMode },
  { "gimbalSetRcTargetingMode", luaMavsdkGimbalSetRcTargetingMode },
  { "gimbalSetGpsPointMode", luaMavsdkGimbalSetGpsPointMode },
  { "gimbalSetSysIdTargetingMode", luaMavsdkGimbalSetSysIdTargetingMode },
  { "gimbalSetPitchYawDeg", luaMavsdkGimbalSetPitchYawDeg },

  { "cameraIsReceiving", luaMavsdkCameraIsReceiving },
  { "cameraIsInitialized", luaMavsdkCameraIsInitialized },
  { "cameraGetInfo", luaMavsdkCameraGetInfo },
  { "cameraGetStatus", luaMavsdkCameraGetStatus },
  { "cameraSetVideoMode", luaMavsdkCameraSetVideoMode },
  { "cameraSetPhotoMode", luaMavsdkCameraSetPhotoMode },
  { "cameraStartVideo", luaMavsdkCameraStartVideo },
  { "cameraStopVideo", luaMavsdkCameraStopVideo },
  { "cameraTakePhoto", luaMavsdkCameraTakePhoto },

  { "getRadioRssi", luaMavsdkGetRadioRssi },
  { "getRadioRemoteRssi", luaMavsdkGetRadioRemoteRssi },
  { "getRadioNoise", luaMavsdkGetRadioNoise },
  { "getRadioRemoteNoise", luaMavsdkGetRadioRemoteNoise },

  { "getAttRollDeg", luaMavsdkGetAttRollDeg },
  { "getAttPitchDeg", luaMavsdkGetAttPitchDeg },
  { "getAttYawDeg", luaMavsdkGetAttYawDeg },

  { "getGpsStatus", luaMavsdkGetGpsStatus }, 		// .fix,.hdop,.sat
  { "getGpsFix", luaMavsdkGetGpsFix },				// GPS_FIX_TYPE_xxx
  { "getGpsHDop", luaMavsdkGetGpsHDop },
  { "getGpsVDop", luaMavsdkGetGpsVDop },
  { "getGpsSat", luaMavsdkGetGpsSat },
  { "getGpsLatLonDeg", luaMavsdkGetGpsLatLonDeg },
  { "getGpsAltitudeAmsl", luaMavsdkGetGpsAltitudeAmsl },
  { "getGpsSpeed", luaMavsdkGetGpsSpeed },
  { "getGpsCourseOverGround", luaMavsdkGetGpsCourseOverGround },

  { "getPositionAltitudeRelative", luaMavsdkGetPositionAltitudeRelative },

  { "getVfrAirSpeed", luaMavsdkGetVfrAirSpeed },
  { "getVfrGroundSpeed", luaMavsdkGetVfrGroundSpeed },
  { "getVfrAltitudeMsl", luaMavsdkGetVfrAltitudeMsl },
  { "getVfrClimbRate", luaMavsdkGetVfrClimbRate },
  { "getVfrHeading", luaMavsdkGetVfrHeading },
  { "getVfrThrottle", luaMavsdkGetVfrThrottle },

  { "getBat1ChargeConsumed", luaMavsdkGetBat1ChargeConsumed },
  { "getBat1EnergyConsumed", luaMavsdkGetBat1EnergyConsumed },
  { "getBat1Temperature", luaMavsdkGetBat1Temperature },
  { "getBat1Voltage", luaMavsdkGetBat1Voltage },
  { "getBat1Current", luaMavsdkGetBat1Current },
  { "getBat1Remaining", luaMavsdkGetBat1Remaining },
  { "getBat1CellCount", luaMavsdkGetBat1CellCount },

  { "getBat2ChargeConsumed", luaMavsdkGetBat2ChargeConsumed },
  { "getBat2EnergyConsumed", luaMavsdkGetBat2EnergyConsumed },
  { "getBat2Temperature", luaMavsdkGetBat2Temperature },
  { "getBat2Voltage", luaMavsdkGetBat2Voltage },
  { "getBat2Current", luaMavsdkGetBat2Current },
  { "getBat2Remaining", luaMavsdkGetBat2Remaining },
  { "getBat2CellCount", luaMavsdkGetBat2CellCount },

  { "getBatCount", luaMavsdkGetBatCount },
#endif
  { NULL, NULL }  /* sentinel */
};

const luaR_value_entry mavsdkConstants[] = {
#if defined(MAVLINK_TELEM)
  { "VEHICLECLASS_GENERIC", MAVSDK_VEHICLECLASS_GENERIC },
  { "VEHICLECLASS_PLANE", MAVSDK_VEHICLECLASS_PLANE },
  { "VEHICLECLASS_COPTER", MAVSDK_VEHICLECLASS_COPTER },
  { "VEHICLECLASS_ROVER", MAVSDK_VEHICLECLASS_ROVER },
  { "VEHICLECLASS_BOAT", MAVSDK_VEHICLECLASS_BOAT },
  { "VEHICLECLASS_SUB", MAVSDK_VEHICLECLASS_SUB },
#endif
  { nullptr, 0 }  /* sentinel */
};

