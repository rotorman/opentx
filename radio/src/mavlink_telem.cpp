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
 * (c) www.olliw.eu, OlliW, OlliW42
 */

#include "opentx.h"


//extern Fifo<uint8_t, MAVLINK_TELEM_TX_FIFO_SIZE> mavlinkTelemTxFifo;
//extern Fifo<uint8_t, MAVLINK_TELEM_RX_FIFO_SIZE> mavlinkTelemRxFifo;


MavlinkTelem mavlinkTelem;


#define FPI         3.141592654f
#define FDEGTORAD   (FPI/180.0f)
#define FRADTODEG   (180.0f/FPI)

#define INCU8(x)  if ((x) < UINT8_MAX) { (x)++; }


// -- TASK handlers --
// tasks can be set directly with SETTASK()
// some tasks don't need immediate execution, or need reliable request
// this is what these handlers are for
// they push the task to a fifo, and also allow to set number of retries and retry rates

void MavlinkTelem::push_task(uint8_t idx, uint32_t task)
{
    struct Task t = {.task = task, .idx = idx};
    _taskFifo.push(t);
}


void MavlinkTelem::pop_and_set_task(void)
{
    struct Task t;
    if (_taskFifo.pop(t)) SETTASK(t.idx, t.task);
}


void MavlinkTelem::set_request(uint8_t idx, uint32_t task, uint8_t retry, tmr10ms_t rate)
{
    push_task(idx, task);

    _request_is_waiting[idx] |= task;

    if (retry == 0) return; //well, if there would be another pending we would not kill it

    int8_t empty_i = -1;

    // first check if request is already pending, at the same time find free slot, to avoid having to loop twice
    for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
        // TODO: should we modify the retry & rate of the pending task?
        if ((_requestList[i].idx == idx) && (_requestList[i].task == task)) return; // already pending, we can get out of here
        if ((empty_i < 0) && !_requestList[i].task) empty_i = i; // free slot
    }

    // if not already pending, add it
    if (empty_i < 0) return; // no free slot

    _requestList[empty_i].task = task;
    _requestList[empty_i].idx = idx;
    _requestList[empty_i].retry = retry;
    _requestList[empty_i].tlast = get_tmr10ms();
    _requestList[empty_i].trate = rate;
}


void MavlinkTelem::clear_request(uint8_t idx, uint32_t task)
{
    for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
        if ((_requestList[i].idx == idx) && (_requestList[i].task == task)) {
            _requestList[i].task = 0;
            _request_is_waiting[idx] &=~ task;
        }
    }
}

//what happens if a clear never comes?
// well, this is what retry = UINT8_MAX says, right

void MavlinkTelem::do_requests(void)
{
    tmr10ms_t tnow = get_tmr10ms();

    for (uint16_t i = 0; i < TASKIDX_MAX; i++) _request_is_waiting[i] = 0;

    for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
        if (!_requestList[i].task) continue;

        _request_is_waiting[_requestList[i].idx] |= _requestList[i].task;

        if ((tnow - _requestList[i].tlast) >= _requestList[i].trate) {
            push_task(_requestList[i].idx, _requestList[i].task);
            _requestList[i].tlast = get_tmr10ms();
            if (_requestList[i].retry < UINT8_MAX) {
                if (_requestList[i].retry) _requestList[i].retry--;
                if (!_requestList[i].retry) _requestList[i].task = 0; // clear request
            }
        }
    }

    if ((tnow - _taskFifo_tlast) > 6) { // 60 ms decimation
        _taskFifo_tlast = tnow;
        // change this, so that it skips tasks with 0, this would allow an easy means to clear tasks also in the Fifo
        if (!_taskFifo.isEmpty()) pop_and_set_task();
    }
}


// -- MAVLink stuff --

bool MavlinkTelem::isInVersionV2(void)
{
	return (_status.flags & MAVLINK_STATUS_FLAG_IN_MAVLINK1) ? false : true;
}


void MavlinkTelem::setOutVersionV2(void)
{
	_status.flags &=~ MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
}


void MavlinkTelem::setOutVersionV1(void)
{
	_status.flags |= MAVLINK_STATUS_FLAG_OUT_MAVLINK1;
}


// -- Generate MAVLink messages --
// these should never be called directly, should only by called by the task handler

void MavlinkTelem::_generateCmdLong(
        uint8_t tsystem, uint8_t tcomponent, uint16_t cmd,
        float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    setOutVersionV2();
    mavlink_msg_command_long_pack(
            _my_sysid, _my_compid, &_msg_out,
            tsystem, tcomponent, cmd, 0, p1, p2, p3, p4, p5, p6, p7
            );
    _txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}

void MavlinkTelem::generateHeartbeat(uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
	setOutVersionV2();
	mavlink_msg_heartbeat_pack(
		  _my_sysid, _my_compid, &_msg_out,
          MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, base_mode, custom_mode, system_status
		  );
	_txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}

void MavlinkTelem::generateParamRequestList(uint8_t tsystem, uint8_t tcomponent)
{
	setOutVersionV2();
	mavlink_msg_param_request_list_pack(
			_my_sysid, _my_compid, &_msg_out,
			tsystem, tcomponent
			);
	_txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}

void MavlinkTelem::generateParamRequestRead(uint8_t tsystem, uint8_t tcomponent, char* param_name)
{
char param_id[16];
    strncpy(param_id, param_name, 16);
    setOutVersionV2();
    mavlink_msg_param_request_read_pack(
            _my_sysid, _my_compid, &_msg_out,
            tsystem, tcomponent, param_id, -1
            );
    _txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}

void MavlinkTelem::generateRequestDataStream(
        uint8_t tsystem, uint8_t tcomponent, uint8_t data_stream, uint16_t rate_hz, uint8_t startstop)
{
    setOutVersionV2();
    mavlink_msg_request_data_stream_pack(
            _my_sysid, _my_compid, &_msg_out,
            tsystem, tcomponent, data_stream, rate_hz, startstop
            );
    _txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}

//ArduPilot: ignores param7
void MavlinkTelem::generateCmdSetMessageInterval(uint8_t tsystem, uint8_t tcomponent, uint8_t msgid, int32_t period_us, uint8_t startstop)
{
  _generateCmdLong(tsystem, tcomponent, MAV_CMD_SET_MESSAGE_INTERVAL, msgid, (startstop) ? period_us : -1.0f);
}

//ArduPilot:
//  base_mode must have MAV_MODE_FLAG_CUSTOM_MODE_ENABLED bit set,
//  custom_mode then determines the mode it will switch to
//  usage of this cmd is thus very likely very flightstack dependent!!
void MavlinkTelem::generateCmdDoSetMode(uint8_t tsystem, uint8_t tcomponent, MAV_MODE base_mode, uint32_t custom_mode)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_DO_SET_MODE, base_mode, custom_mode);
}

//ArduPilot: supports a param3 which is not in the specs, ignores param1,4,5,6
// param3 = horizontal navigation by pilot acceptable
void MavlinkTelem::generateCmdNavTakeoff(uint8_t tsystem, uint8_t tcomponent, float alt_m, bool hor_nav_by_pilot)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_NAV_TAKEOFF, 0,0, (hor_nav_by_pilot) ? 1.0f : 0.0f, 0,0,0, alt_m);
}

// speed type 0=Airspeed, 1=Ground Speed, 2=Climb Speed, 3=Descent Speed
//ArduPilot: ignores param3 =  Throttle and param4 = Relative
void MavlinkTelem::generateCmdDoChangeSpeed(uint8_t tsystem, uint8_t tcomponent, float speed_mps, uint16_t speed_type, bool relative)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_DO_CHANGE_SPEED, speed_type, speed_mps, -1, (relative) ? 1.0f : 0.0f);
}

//ArduPilot: current = 2 or 3
void MavlinkTelem::generateMissionItemInt(uint8_t tsystem, uint8_t tcomponent,
        uint8_t frame, uint16_t cmd, uint8_t current, int32_t lat, int32_t lon, float alt_m)
{
    setOutVersionV2();
    mavlink_msg_mission_item_int_pack(
            _my_sysid, _my_compid, &_msg_out,
            tsystem, tcomponent,
            0, frame, cmd, current, 0, 0.0f, 0.0f, 0.0f, 0.0f, lat, lon, alt_m, MAV_MISSION_TYPE_MISSION
//uint8_t target_system, uint8_t target_component, uint16_t seq, uint8_t frame, uint16_t command, uint8_t current,
//uint8_t autocontinue, float param1, float param2, float param3, float param4, int32_t x, int32_t y, float z, uint8_t mission_type)
            );
    _txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}

void MavlinkTelem::generateSetPositionTargetGlobalInt(uint8_t tsystem, uint8_t tcomponent,
        uint8_t frame, uint16_t type_mask,
        int32_t lat, int32_t lon, float alt, float vx, float vy, float vz, float yaw_rad, float yaw_rad_rate)
{
    setOutVersionV2();
    mavlink_msg_set_position_target_global_int_pack(
            _my_sysid, _my_compid, &_msg_out,
            0, //uint32_t time_boot_ms,
            tsystem, tcomponent,
            frame, type_mask,
            lat, lon, alt, vx, vy, vz, 0.0f, 0.0f, 0.0f, yaw_rad, yaw_rad_rate // alt in m, v in m/s, yaw in rad
            );
    _txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}

// yaw must be in range 0..360
void MavlinkTelem::generateCmdConditionYaw(uint8_t tsystem, uint8_t tcomponent, float yaw_deg, float yaw_deg_rate, int8_t dir, bool rel)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_CONDITION_YAW, yaw_deg, yaw_deg_rate, (dir>0)?1.0f:-1.0f, (rel)?1.0f:0.0f);
}


void MavlinkTelem::generateCmdRequestCameraInformation(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_REQUEST_CAMERA_INFORMATION, 1, 0,0,0,0,0,0);
}

void MavlinkTelem::generateCmdRequestCameraSettings(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_REQUEST_CAMERA_SETTINGS, 1, 0,0,0,0,0,0);
}

void MavlinkTelem::generateCmdRequestStorageInformation(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_REQUEST_STORAGE_INFORMATION, 0, 1, 0,0,0,0,0);
}

void MavlinkTelem::generateCmdRequestCameraCapturesStatus(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_REQUEST_CAMERA_CAPTURE_STATUS, 1, 0,0,0,0,0,0);
}

void MavlinkTelem::generateCmdSetCameraMode(uint8_t tsystem, uint8_t tcomponent, uint8_t mode)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_SET_CAMERA_MODE, 0, mode, 0,0,0,0,0);
}

void MavlinkTelem::generateCmdImageStartCapture(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 1, 0, 0,0,0);
}

void MavlinkTelem::generateCmdVideoStartCapture(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_VIDEO_START_CAPTURE, 0, 0.2f, 0,0,0,0,0);
}

void MavlinkTelem::generateCmdVideoStopCapture(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_VIDEO_STOP_CAPTURE, 0,0,0,0,0,0,0);
}


void MavlinkTelem::generateCmdDoMountConfigure(uint8_t tsystem, uint8_t tcomponent, uint8_t mode)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_DO_MOUNT_CONFIGURE, mode, 0,0,0,0,0,0);
}

//ArduPilot: if a mount has no pan control, then this will also yaw the copter in guided mode overwriting _fixed_yaw !!
void MavlinkTelem::generateCmdDoMountControl(uint8_t tsystem, uint8_t tcomponent, float pitch_deg, float yaw_deg)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_DO_MOUNT_CONTROL,
            pitch_deg, 0.0, yaw_deg, 0,0,0, MAV_MOUNT_MODE_MAVLINK_TARGETING);
}


// -- Main task handler --

void MavlinkTelem::doTask(void)
{
	tmr10ms_t tnow = get_tmr10ms();

	bool tick_1Hz = false;

	if ((tnow - _my_heartbeat_tlast) > 100) { //1 sec
		_my_heartbeat_tlast = tnow;
		SETTASK(TASK_ME, TASK_SENDMYHEARTBEAT);

		msg_rx_persec = _msg_rx_persec_cnt;
		_msg_rx_persec_cnt = 0;
		bytes_rx_persec = _bytes_rx_persec_cnt;
		_bytes_rx_persec_cnt = 0;

		tick_1Hz = true;
	}

	if (!isSystemIdValid()) return;

	// we need to wait until at least one heartbeat was send out before requesting data streams
	if (autopilot.requests_triggered) {
		if (tick_1Hz) autopilot.requests_triggered++;
		if (autopilot.requests_triggered > 3) { // wait for 3 heartbeats
			autopilot.requests_triggered = 0;
			requestDataStreamFromAutopilot();
		}
	}

	// we wait until at least one heartbeat was send out, and autopilot requests have been done
    if (camera.compid && camera.requests_triggered && !autopilot.requests_triggered) {
		if (tick_1Hz) camera.requests_triggered++;
		if (camera.requests_triggered > 1) { // wait for the next heartbeat
			camera.requests_triggered = 0;
	    	set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_INFORMATION, 10, 200); //10x every ca 2sec
	    	set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS, 10, 205);
	    	set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS, 10, 210);
	    	set_request(TASK_CAMERA, TASK_SENDREQUEST_STORAGE_INFORMATION, 10, 215);
    	}
    }

    if (!autopilot.is_initialized) {
        autopilot.is_initialized = (autopilot.requests_waiting_mask == 0);
    }

    if (!camera.is_initialized) {
    	camera.is_initialized = ((camera.requests_waiting_mask & CAMERA_REQUESTWAITING_ALL) == 0);
    }

    // do pending requests
    do_requests();

    // send out pending messages
	if ((_txcount == 0) && TASK_IS_PENDING()) {
		if (_task[TASK_ME] & TASK_SENDMYHEARTBEAT) {
	        RESETTASK(TASK_ME,TASK_SENDMYHEARTBEAT);
	        uint8_t base_mode = MAV_MODE_PREFLIGHT | MAV_MODE_FLAG_CUSTOM_MODE_ENABLED | MAV_MODE_FLAG_SAFETY_ARMED;
	        uint8_t system_status = MAV_STATE_UNINIT | MAV_STATE_ACTIVE;
            uint32_t custom_mode = 0;
            generateHeartbeat(base_mode, custom_mode, system_status);
	        return; //do only one per loop
	    }

	    // autopilot tasks
        if (_task[TASK_AUTOPILOT] & TASK_SENDCMD_DO_SET_MODE) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDCMD_DO_SET_MODE);
            generateCmdDoSetMode(_sysid, autopilot.compid, (MAV_MODE)_tcsm_base_mode, _tcsm_custom_mode);
            return; //do only one per loop
        }
        if (_task[TASK_AUTOPILOT] & TASK_SENDCMD_DO_CHANGE_SPEED) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDCMD_DO_CHANGE_SPEED);
            generateCmdDoChangeSpeed(_sysid, autopilot.compid, _tccs_speed_mps, _tccs_speed_type, true);
            return; //do only one per loop
        }
        if (_task[TASK_AUTOPILOT] & TASK_SENDMSG_MISSION_ITEM_INT) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDMSG_MISSION_ITEM_INT);
            generateMissionItemInt(_sysid, autopilot.compid, _tmii_frame, _tmii_cmd, _tmii_current,
                    _tmii_lat, _tmii_lon, _tmii_alt_m);
            return; //do only one per loop
        }
        if (_task[TASK_AUTOPILOT] & TASK_SENDMSG_SET_POSITION_TARGET_GLOBAL_INT) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDMSG_SET_POSITION_TARGET_GLOBAL_INT);
            generateSetPositionTargetGlobalInt(_sysid, autopilot.compid, _t_coordinate_frame, _t_type_mask,
                    _t_lat, _t_lon, _t_alt, _t_vx, _t_vy, _t_vz, _t_yaw_rad, _t_yaw_rad_rate);
            return; //do only one per loop
        }
        if (_task[TASK_AUTOPILOT] & TASK_SENDCMD_NAV_TAKEOFF) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDCMD_NAV_TAKEOFF);
            generateCmdNavTakeoff(_sysid, autopilot.compid, _tcnt_alt_m, 1);
            return; //do only one per loop
        }
        if (_task[TASK_AUTOPILOT] & TASK_SENDCMD_CONDITION_YAW) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDCMD_CONDITION_YAW);
            generateCmdConditionYaw(_sysid, autopilot.compid, _tccy_yaw_deg, 0.0f, _tccy_dir, _tccy_relative);
            return; //do only one per loop
        }

		if (_task[TASK_AUTOPILOT] & TASK_SENDMSG_PARAM_REQUEST_LIST) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDMSG_PARAM_REQUEST_LIST);
	        generateParamRequestList(_sysid, autopilot.compid);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_RAW_SENSORS) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_RAW_SENSORS);
//XX	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_RAW_SENSORS, 2, 1);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_EXTENDED_STATUS) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_EXTENDED_STATUS);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_POSITION) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_POSITION);
//XX	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_POSITION, 4, 1); // do faster, 4 Hz
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_EXTRA1) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_EXTRA1);
//XX	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_EXTRA1, 4, 1); // do faster, 4 Hz
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_EXTRA2) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_EXTRA2);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_EXTRA2, 2, 1);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_EXTRA3) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_EXTRA3);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_EXTRA3, 2, 1);
	        return; //do only one per loop
		}

        if (_task[TASK_AUTOPILOT] & TASK_SENDCMD_REQUEST_ATTITUDE) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDCMD_REQUEST_ATTITUDE);
            generateCmdSetMessageInterval(_sysid, autopilot.compid, MAVLINK_MSG_ID_ATTITUDE, 100000, 1);
            return; //do only one per loop
        }
        if (_task[TASK_AUTOPILOT] & TASK_SENDCMD_REQUEST_GLOBAL_POSITION_INT) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDCMD_REQUEST_GLOBAL_POSITION_INT);
            generateCmdSetMessageInterval(_sysid, autopilot.compid, MAVLINK_MSG_ID_GLOBAL_POSITION_INT, 100000, 1);
            return; //do only one per loop
        }

        if (_task[TASK_AP] & TASK_ARDUPILOT_REQUESTBANNER) { //MAV_CMD_DO_SEND_BANNER
            RESETTASK(TASK_AP, TASK_ARDUPILOT_REQUESTBANNER);
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_DO_SEND_BANNER);
            return; //do only one per loop
        }
        if (_task[TASK_AP] & TASK_ARDUPILOT_ARM) { //MAV_CMD_COMPONENT_ARM_DISARM
            RESETTASK(TASK_AP, TASK_ARDUPILOT_ARM);
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_COMPONENT_ARM_DISARM, 1.0f);
            return; //do only one per loop
        }
        if (_task[TASK_AP] & TASK_ARDUPILOT_DISARM) { //MAV_CMD_COMPONENT_ARM_DISARM
            RESETTASK(TASK_AP, TASK_ARDUPILOT_DISARM);
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_COMPONENT_ARM_DISARM, 0.0f);
            return; //do only one per loop
        }
        if (_task[TASK_AP] & TASK_ARDUPILOT_COPTER_TAKEOFF) { //MAV_CMD_NAV_TAKEOFF
            RESETTASK(TASK_AP, TASK_ARDUPILOT_COPTER_TAKEOFF);
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_NAV_TAKEOFF, 0,0, 0.0f, 0,0,0, _tact_takeoff_alt_m); //must_navigate = true
            return; //do only one per loop
        }
        if (_task[TASK_AP] & TASK_ARDUPILOT_LAND) { //MAV_CMD_NAV_LAND
            RESETTASK(TASK_AP, TASK_ARDUPILOT_LAND);
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_NAV_LAND);
            return; //do only one per loop
        }
        if (_task[TASK_AP] & TASK_ARDUPILOT_COPTER_FLYCLICK) {
            RESETTASK(TASK_AP, TASK_ARDUPILOT_COPTER_FLYCLICK);
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_SOLO_BTN_FLY_CLICK);
            return; //do only one per loop
        }
        if (_task[TASK_AP] & TASK_ARDUPILOT_COPTER_FLYHOLD) {
            RESETTASK(TASK_AP, TASK_ARDUPILOT_COPTER_FLYHOLD);
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_SOLO_BTN_FLY_HOLD, _tacf_takeoff_alt_m);
            return; //do only one per loop
        }
        if (_task[TASK_AP] & TASK_ARDUPILOT_COPTER_FLYPAUSE) {
            RESETTASK(TASK_AP, TASK_ARDUPILOT_COPTER_FLYPAUSE);
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_SOLO_BTN_PAUSE_CLICK, 0.0f); //shoot = no
            return; //do only one per loop
        }

	    // camera tasks
		if (_task[TASK_CAMERA] & TASK_SENDCMD_SET_CAMERA_VIDEO_MODE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_SET_CAMERA_VIDEO_MODE);
	        if (!camera.compid) return;
	        generateCmdSetCameraMode(_sysid, camera.compid, CAMERA_MODE_VIDEO);
	        set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS, 2);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDCMD_SET_CAMERA_PHOTO_MODE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_SET_CAMERA_PHOTO_MODE);
            if (!camera.compid) return;
	        generateCmdSetCameraMode(_sysid, camera.compid, CAMERA_MODE_IMAGE);
            set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS, 2);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDCMD_IMAGE_START_CAPTURE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_IMAGE_START_CAPTURE);
            if (!camera.compid) return;
	        generateCmdImageStartCapture(_sysid, camera.compid);
            set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS, 2);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDCMD_VIDEO_START_CAPTURE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_VIDEO_START_CAPTURE);
            if (!camera.compid) return;
	        generateCmdVideoStartCapture(_sysid, camera.compid);
            set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS, 2);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDCMD_VIDEO_STOP_CAPTURE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_VIDEO_STOP_CAPTURE);
            if (!camera.compid) return;
	        generateCmdVideoStopCapture(_sysid, camera.compid);
            set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS, 2);
	        return; //do only one per loop
		}

		// the sequence here defines the startup sequence
		if (_task[TASK_CAMERA] & TASK_SENDREQUEST_CAMERA_INFORMATION) {
	        RESETTASK(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_INFORMATION);
	        if (camera.compid) generateCmdRequestCameraInformation(_sysid, camera.compid);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDREQUEST_CAMERA_SETTINGS) {
	        RESETTASK(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS);
	        if (camera.compid) generateCmdRequestCameraSettings(_sysid, camera.compid);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS) {
	        RESETTASK(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS);
	        if (camera.compid) generateCmdRequestCameraCapturesStatus(_sysid, camera.compid);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDREQUEST_STORAGE_INFORMATION) {
	        RESETTASK(TASK_CAMERA, TASK_SENDREQUEST_STORAGE_INFORMATION);
	        if (camera.compid) generateCmdRequestStorageInformation(_sysid, camera.compid);
	        return; //do only one per loop
		}

	    // gimbal tasks
		if (_task[TASK_GIMBAL] & TASK_SENDCMD_DO_MOUNT_CONFIGURE) {
	        RESETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_MOUNT_CONFIGURE);
	        generateCmdDoMountConfigure(_sysid, autopilot.compid, _t_gimbal_mode);
	        return; //do only one per loop
		}
		if (_task[TASK_GIMBAL] & TASK_SENDCMD_DO_MOUNT_CONTROL) {
	        RESETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_MOUNT_CONTROL);
	        generateCmdDoMountControl(_sysid, autopilot.compid, _t_gimbal_pitch_deg, _t_gimbal_yaw_deg);
	        return; //do only one per loop
		}
	}
}


// -- Mavsdk Convenience Task Wrapper --

void MavlinkTelem::apSetFlightMode(uint32_t ap_flight_mode)
{
    _tcsm_base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    _tcsm_custom_mode = ap_flight_mode;
    SETTASK(TASK_AUTOPILOT, TASK_SENDCMD_DO_SET_MODE);
}

void MavlinkTelem::apSetGroundSpeed(float speed)
{
    _tccs_speed_mps = speed;
    _tccs_speed_type = 1;
    SETTASK(TASK_AUTOPILOT, TASK_SENDCMD_DO_CHANGE_SPEED);
}

void MavlinkTelem::apSimpleGotoPosAlt(int32_t lat, int32_t lon, float alt)
{
    _tmii_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    _tmii_cmd = MAV_CMD_NAV_WAYPOINT;
    _tmii_current = 2;
    _tmii_lat = lat; _tmii_lon = lon; _tmii_alt_m = alt;
    SETTASK(TASK_AUTOPILOT, TASK_SENDMSG_MISSION_ITEM_INT);
}

//alt and yaw can be NAN if they should be ignored
// this function is not very useful as it really moves very slowly
void MavlinkTelem::apGotoPosAltYawDeg(int32_t lat, int32_t lon, float alt, float yaw)
{
    _t_coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    //_t_type_mask = 0x09F8;
    _t_type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                   POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    if (isnan(alt)) { _t_type_mask |= POSITION_TARGET_TYPEMASK_Z_IGNORE; alt = 1.0f; }
    if (isnan(yaw)) { _t_type_mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE; yaw = 0.0f; }
    _t_lat = lat; _t_lon = lon;
    _t_alt = alt; // m
    _t_vx = _t_vy = _t_vz = 0.0f;
    _t_yaw_rad = yaw * FDEGTORAD; // rad
    _t_yaw_rad_rate = 0.0f;
    SETTASK(TASK_AUTOPILOT, TASK_SENDMSG_SET_POSITION_TARGET_GLOBAL_INT);
}

void MavlinkTelem::apGotoPosAltVel(int32_t lat, int32_t lon, float alt, float vx, float vy, float vz)
{
    _t_coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    _t_type_mask = POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                   POSITION_TARGET_TYPEMASK_YAW_IGNORE |
                   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    _t_lat = lat; _t_lon = lon;
    _t_alt = alt; // m
    _t_vx = vx; _t_vy = vy; _t_vz = vz; // m/s
    _t_yaw_rad = _t_yaw_rad_rate = 0.0f; // rad
    SETTASK(TASK_AUTOPILOT, TASK_SENDMSG_SET_POSITION_TARGET_GLOBAL_INT);
}

bool MavlinkTelem::apMoveToPosAltWithSpeed(int32_t lat, int32_t lon, float alt, float speed, bool xy)
{
    // TODO: check if we have a valid position
    // mimic ArduCopter's position_ok() when it is in armed state
    bool ok = (ekf.flags & MAVAP_EKF_POS_HORIZ_ABS) && !(ekf.flags & MAVAP_EKF_CONST_POS_MODE);
    if (!ok) return false;
    //we grab the current location
    // from this we calculate the normalized direction vector
    // which is then multiplied by speed to get the velocity vector
    // we use flat earth approximation
    //   xScale = cos(rad(lat0 * 1.0e-7))
    //   x = rad((lon - lon0) * 1.0e-7) * xScale * R
    //   y = rad((lat - lat0) * 1.0e-7) * R
    // in order to get xscale and stay within float we are happy with ca 4 digits accuracy
    float xscale = cosf( (float)((lat + gposition.lat)/20000) * (1.0E-3f * FPI/180.0f) );
    float dx = (float)( lon - gposition.lon ) * xscale * (0.6371f); //m
    float dy = (float)( lat - gposition.lat ) * (0.6371f); //m
    float dz = (xy) ? 0.0f : (alt - 0.001f * (float)gposition.relative_alt_mm); //m
    float dist = sqrtf(dx*dx + dy*dy + dz*dz);
    if (dist == 0.0f ) return false; // if the distance is zero, we can skip it
    apGotoPosAltVel(lat, lon, alt, (dx/dist)*speed, (dy/dist)*speed, (dz/dist)*speed);
    return true;
}


//note, we can enter negative yaw here, sign determines direction
void MavlinkTelem::apSetYawDeg(float yaw, bool relative)
{
    if (relative) {
        _tccy_relative = 1.0f;
        if (yaw < 0.0f){ _tccy_dir = -1.0f; yaw = -yaw; } else{ _tccy_dir = 1.0f; }
    } else {
        _tccy_relative = 0.0f;
        _tccy_dir = 0.0f;
    }
    float res = fmodf(yaw, 360.0f);
    if (res < 0.0f) res += 360.0f;
    _tccy_yaw_deg = res;  // is in deg, must be in range [0..360]
    SETTASK(TASK_AUTOPILOT, TASK_SENDCMD_CONDITION_YAW);
}


// -- Handle incoming MAVLink messages, which are for the Camera --

void MavlinkTelem::handleMessageCamera(void)
{
	switch (_msg.msgid) {

	case MAVLINK_MSG_ID_HEARTBEAT: {
		mavlink_heartbeat_t payload;
		mavlink_msg_heartbeat_decode(&_msg, &payload);
		camera.system_status = payload.system_status;
		camera.is_armed = (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false;
        camera.is_standby = (payload.system_status <= MAV_STATE_STANDBY) ? true : false;
        camera.is_critical = (payload.system_status >= MAV_STATE_CRITICAL) ? true : false;
        INCU8(camera.updated);
		camera.is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT;
		}break;

	case MAVLINK_MSG_ID_CAMERA_INFORMATION: {
		mavlink_camera_information_t payload;
		mavlink_msg_camera_information_decode(&_msg, &payload);
		memset(cameraInfo.vendor_name, 0, 32+1);
		memcpy(cameraInfo.vendor_name, payload.vendor_name, 32);
		memset(cameraInfo.model_name, 0, 32+1);
		memcpy(cameraInfo.model_name, payload.model_name, 32);
		cameraInfo.flags = payload.flags;
		cameraInfo.has_video = (cameraInfo.flags & CAMERA_CAP_FLAGS_CAPTURE_VIDEO);
		cameraInfo.has_photo = (cameraInfo.flags & CAMERA_CAP_FLAGS_CAPTURE_IMAGE);
		cameraInfo.has_modes = (cameraInfo.flags & CAMERA_CAP_FLAGS_HAS_MODES);
        clear_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_INFORMATION);
        camera.requests_waiting_mask &=~ CAMERA_REQUESTWAITING_CAMERA_INFORMATION;
		}break;

	case MAVLINK_MSG_ID_CAMERA_SETTINGS: {
		mavlink_camera_settings_t payload;
		mavlink_msg_camera_settings_decode(&_msg, &payload);
		cameraStatus.mode = (payload.mode_id == CAMERA_MODE_IMAGE) ? CAMERA_MODE_IMAGE : CAMERA_MODE_VIDEO;
        clear_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS);
        camera.requests_waiting_mask &=~ CAMERA_REQUESTWAITING_CAMERA_SETTINGS;
		}break;

	case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: {
		mavlink_camera_capture_status_t payload;
		mavlink_msg_camera_capture_status_decode(&_msg, &payload);
		cameraStatus.recording_time_ms = payload.recording_time_ms;
		cameraStatus.available_capacity_MiB = payload.available_capacity;
		cameraStatus.video_on = (payload.video_status > 0);
		cameraStatus.photo_on = (payload.image_status > 0); //0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress
        clear_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS);
        camera.requests_waiting_mask &=~ CAMERA_REQUESTWAITING_CAMERA_CAPTURE_STATUS;
		}break;

	case MAVLINK_MSG_ID_STORAGE_INFORMATION: {
		mavlink_storage_information_t payload;
		mavlink_msg_storage_information_decode(&_msg, &payload);
        if (payload.status == STORAGE_STATUS_READY) {
			cameraInfo.total_capacity_MiB = payload.total_capacity;
			cameraStatus.available_capacity_MiB = payload.available_capacity;
		} else {
			cameraInfo.total_capacity_MiB = NAN;
			cameraStatus.available_capacity_MiB = NAN;
		}
        clear_request(TASK_CAMERA, TASK_SENDREQUEST_STORAGE_INFORMATION);
		}break;

	case MAVLINK_MSG_ID_BATTERY_STATUS: {
		mavlink_battery_status_t payload;
		mavlink_msg_battery_status_decode(&_msg, &payload);
    	int32_t voltage = 0;
    	bool has_voltage = false;
    	for (uint8_t i=0; i<10; i++) {
    		if (payload.voltages[i] != UINT16_MAX) {
    			voltage += payload.voltages[i]; //uint16_t mV, UINT16_MAX if not known
    			has_voltage = true;
    		}
    	}
  		cameraStatus.battery_voltage_V = (has_voltage) ? 0.001f * (float)voltage : NAN;
    	cameraStatus.battery_remaining_pct = payload.battery_remaining; // -1 if not known
		}break;

	}
}


// -- Handle incoming MAVLink messages, which are for the Gimbal --

void MavlinkTelem::handleMessageGimbal(void)
{
	switch (_msg.msgid) {

	case MAVLINK_MSG_ID_HEARTBEAT: {
		mavlink_heartbeat_t payload;
		mavlink_msg_heartbeat_decode(&_msg, &payload);
		gimbal.system_status = payload.system_status;
		gimbal.custom_mode = payload.custom_mode;
		gimbal.is_armed = (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false;
        gimbal.is_standby = (payload.system_status <= MAV_STATE_STANDBY) ? true : false;
        gimbal.is_critical = (payload.system_status >= MAV_STATE_CRITICAL) ? true : false;
		gimbal.prearm_ok = (payload.custom_mode & 0x80000000) ? false : true;
        INCU8(gimbal.updated);
        gimbal.is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT;
		}break;

    case MAVLINK_MSG_ID_ATTITUDE: {
    	mavlink_attitude_t payload;
        mavlink_msg_attitude_decode(&_msg, &payload);
        gimbalAtt.roll_deg = payload.roll * FRADTODEG;
        gimbalAtt.pitch_deg = payload.pitch * FRADTODEG;
        gimbalAtt.yaw_deg_relative = payload.yaw * FRADTODEG;
        gimbalAtt.yaw_deg_absolute = gimbalAtt.yaw_deg_relative + att.yaw_rad * FRADTODEG;
        if (gimbalAtt.yaw_deg_absolute > 180.0f) gimbalAtt.yaw_deg_absolute -= 360.0f;
        if (gimbalAtt.yaw_deg_absolute < -180.0f) gimbalAtt.yaw_deg_absolute += 360.0f;
        INCU8(gimbalAtt.updated);
		}break;

    case MAVLINK_MSG_ID_MOUNT_STATUS: {
    	mavlink_mount_status_t payload;
        mavlink_msg_mount_status_decode(&_msg, &payload);
        gimbalAtt.roll_deg = ((float)payload.pointing_b * 0.01f);
        gimbalAtt.pitch_deg = ((float)payload.pointing_a * 0.01f);
        gimbalAtt.yaw_deg_relative = ((float)payload.pointing_c * 0.01f);
        gimbalAtt.yaw_deg_absolute = gimbalAtt.yaw_deg_relative + att.yaw_rad * FRADTODEG;
        if (gimbalAtt.yaw_deg_absolute > 180.0f) gimbalAtt.yaw_deg_absolute -= 360.0f;
        if (gimbalAtt.yaw_deg_absolute < -180.0f) gimbalAtt.yaw_deg_absolute += 360.0f;
        INCU8(gimbalAtt.updated);
		}break;
	}
}


// -- Handle incoming MAVLink messages, which are for the Autopilot --

void MavlinkTelem::handleMessageAutopilot(void)
{
	switch (_msg.msgid) {

	case MAVLINK_MSG_ID_HEARTBEAT: {
		mavlink_heartbeat_t payload;
		mavlink_msg_heartbeat_decode(&_msg, &payload);
		flightmode = payload.custom_mode;
		autopilot.system_status = payload.system_status;
		autopilot.is_armed = (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false;
		autopilot.is_standby = (payload.system_status <= MAV_STATE_STANDBY) ? true : false;
        autopilot.is_critical = (payload.system_status >= MAV_STATE_CRITICAL) ? true : false;
        INCU8(autopilot.updated);
		autopilot.is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT;
		}break;

    case MAVLINK_MSG_ID_ATTITUDE: {
    	mavlink_attitude_t payload;
        mavlink_msg_attitude_decode(&_msg, &payload);
        att.roll_rad = payload.roll;
        att.pitch_rad = payload.pitch;
        att.yaw_rad = payload.yaw;
        INCU8(att.updated);
        clear_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA1);
        autopilot.requests_waiting_mask &=~ AUTOPILOT_REQUESTWAITING_ATTITUDE;
		}break;

    case MAVLINK_MSG_ID_GPS_RAW_INT: {
    	mavlink_gps_raw_int_t payload;
        mavlink_msg_gps_raw_int_decode(&_msg, &payload);
        gps1.fix = payload.fix_type;
        gps1.sat = payload.satellites_visible;
        gps1.hdop = payload.eph;
        gps1.vdop = payload.epv;
        gps1.lat = payload.lat;
        gps1.lon = payload.lon;
        gps1.alt_mm = payload.alt;
        gps1.vel_cmps = payload.vel;
        gps1.cog_cdeg = payload.cog;
        INCU8(gps1.updated);
        gps_instancemask |= 0x01;
        clear_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTENDED_STATUS);
        autopilot.requests_waiting_mask &=~ AUTOPILOT_REQUESTWAITING_GPS_RAW_INT;
        if (g_model.mavlinkMimicSensors) {
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, GPS_ALT_FIRST_ID, 0, 10, (int32_t)(payload.alt), UNIT_METERS, 3);
            if (payload.vel != UINT16_MAX)
                setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, GPS_SPEED_FIRST_ID, 0, 11, (int32_t)(payload.vel), UNIT_METERS, 2);
            // payload.cog //uint16 cdeg
            // { GPS_COURS_FIRST_ID, GPS_COURS_LAST_ID, 0, ZSTR_HDG, UNIT_DEGREE, 2 },
            // { GPS_LONG_LATI_FIRST_ID, GPS_LONG_LATI_LAST_ID, 0, ZSTR_GPS, UNIT_GPS, 0 },
        }
		}break;

    case MAVLINK_MSG_ID_GPS2_RAW: {
        mavlink_gps2_raw_t payload;
        mavlink_msg_gps2_raw_decode(&_msg, &payload);
        gps2.fix = payload.fix_type;
        gps2.sat = payload.satellites_visible;
        gps2.hdop = payload.eph;
        gps2.vdop = payload.epv;
        gps2.lat = payload.lat;
        gps2.lon = payload.lon;
        gps2.alt_mm = payload.alt;
        gps2.vel_cmps = payload.vel;
        gps2.cog_cdeg = payload.cog;
        INCU8(gps2.updated);
        gps_instancemask |= 0x02;
        }break;

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
        mavlink_global_position_int_t payload;
        mavlink_msg_global_position_int_decode(&_msg, &payload);
        gposition.lat = payload.lat;
        gposition.lon = payload.lon;
        gposition.alt_mm = payload.alt;
        gposition.relative_alt_mm = payload.relative_alt;
        gposition.vx_cmps = payload.vx;
        gposition.vy_cmps = payload.vy;
        gposition.vz_cmps = payload.vz;
        gposition.hdg_cdeg = payload.hdg;
        INCU8(gposition.updated);
        clear_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_POSITION);
        autopilot.requests_waiting_mask &=~ AUTOPILOT_REQUESTWAITING_GLOBAL_POSITION_INT;
        }break;

    case MAVLINK_MSG_ID_VFR_HUD: {
    	mavlink_vfr_hud_t payload;
        mavlink_msg_vfr_hud_decode(&_msg, &payload);
    	vfr.airspd_mps = payload.airspeed;
    	vfr.groundspd_mps = payload.groundspeed;
    	vfr.alt_m = payload.alt;
    	vfr.climbrate_mps = payload.climb;
    	vfr.heading_deg = payload.heading;
    	vfr.thro_pct = payload.throttle;
        INCU8(vfr.updated);
        clear_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA2);
        autopilot.requests_waiting_mask &=~ AUTOPILOT_REQUESTWAITING_VFR_HUD;
        if (g_model.mavlinkMimicSensors) {
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, ALT_FIRST_ID, 0, 13, (int32_t)(payload.alt * 100.0f), UNIT_METERS, 2);
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, VARIO_FIRST_ID, 0, 14, (int32_t)(payload.climb * 100.0f), UNIT_METERS_PER_SECOND, 2);
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, AIR_SPEED_FIRST_ID, 0, 15, (int32_t)(payload.airspeed * 100.0f), UNIT_METERS_PER_SECOND, 2);
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, GPS_COURS_FIRST_ID, 0, 16, (int32_t)payload.heading * 10, UNIT_DEGREE, 1);
        }
		}break;

    /* let's use BATTERY_STATUS, is nearly the same thing
    case MAVLINK_MSG_ID_SYS_STATUS: {
    	mavlink_sys_status_t payload;
    	mavlink_msg_sys_status_decode(&_msg, &payload);
    	// voltage_battery	uint16_t	mV
    	// current_battery	int16_t	cA
    	// battery_remaining	int8_t	%
    	}break; */

    case MAVLINK_MSG_ID_BATTERY_STATUS: {
    	mavlink_battery_status_t payload;
    	mavlink_msg_battery_status_decode(&_msg, &payload);
    	int32_t voltage = 0;
    	int8_t cellcount = 0;
    	bool validcellcount = true;
    	for (uint8_t i=0; i<10; i++) {
    		if (payload.voltages[i] != UINT16_MAX){
    			voltage += payload.voltages[i]; //uint16_t mV, UINT16_MAX if not known
    			if (payload.voltages[i] > 5000) validcellcount = false;
    			cellcount++;
    		}
    	}
    	if (!validcellcount) cellcount = -1;
        if (payload.id == 0) {
    		bat1.charge_consumed_mAh = payload.current_consumed; // mAh, -1 if not known
    		bat1.energy_consumed_hJ = payload.energy_consumed; // 0.1 kJ, -1 if not known
    		bat1.temperature_cC = payload.temperature; // centi-degrees C°, INT16_MAX if not known
    		bat1.voltage_mV = voltage; // mV
    		bat1.current_cA = payload.current_battery; // 10*mA, -1 if not known
    		//bat1.function = payload.battery_function;
    		//bat1.type = payload.type;
    		bat1.remaining_pct = payload.battery_remaining; //(0%: 0, 100%: 100), -1 if not knwon
    		bat1.cellcount = cellcount;
            INCU8(bat1.updated);
        }
        if (payload.id == 1) {
    		bat2.charge_consumed_mAh = payload.current_consumed; // mAh, -1 if not known
    		bat2.energy_consumed_hJ = payload.energy_consumed; // 0.1 kJ, -1 if not known
    		bat2.temperature_cC = payload.temperature; // centi-degrees C°, INT16_MAX if not known
    		bat2.voltage_mV = voltage; // mV
    		bat2.current_cA = payload.current_battery; // 10*mA, -1 if not known
    		bat2.remaining_pct = payload.battery_remaining; //(0%: 0, 100%: 100), -1 if not knwon
    		bat2.cellcount = cellcount;
            INCU8(bat2.updated);
        }
        if (payload.id < 8) bat_instancemask |= (1 << payload.id);
        clear_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA3);
        autopilot.requests_waiting_mask &=~ AUTOPILOT_REQUESTWAITING_BATTERY_STATUS;
        if (g_model.mavlinkMimicSensors) {
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, BATT_ID, 0, 17, voltage/100, UNIT_VOLTS, 1);
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, VFAS_FIRST_ID, 0, 18, voltage/10, UNIT_VOLTS, 2);
            int32_t current_battery = payload.current_battery / 10; //int16_t  cA
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, CURR_FIRST_ID, 0, 19, current_battery, UNIT_AMPS, 1);
            //  { CELLS_FIRST_ID, CELLS_LAST_ID, 0, ZSTR_CELLS, UNIT_CELLS, 2 },
            // { RBOX_BATT1_FIRST_ID, RBOX_BATT1_LAST_ID, 0, ZSTR_BATT1_VOLTAGE, UNIT_VOLTS, 3 },
            // { RBOX_BATT1_FIRST_ID, RBOX_BATT1_LAST_ID, 1, ZSTR_BATT1_CURRENT, UNIT_AMPS, 2 },
            // { RBOX_CNSP_FIRST_ID, RBOX_CNSP_LAST_ID, 0, ZSTR_BATT1_CONSUMPTION, UNIT_MAH, 0 },
        }
    	}break;

    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t payload;
        mavlink_msg_statustext_decode(&_msg, &payload);
        payload.text[49] = '\0'; //terminate it properly, never mind losing the last char
        statustext.fifo.push(payload);
        INCU8(statustext.updated);
        }break;

    case MAVLINK_MSG_ID_EKF_STATUS_REPORT: {
        mavlink_ekf_status_report_t payload;
        mavlink_msg_ekf_status_report_decode(&_msg, &payload);
        //we don't really need the other fields
        ekf.flags = payload.flags;
        INCU8(ekf.updated);
        }break;

    };
}


// -- Main handler for incoming MAVLink messages --

void MavlinkTelem::handleMessage(void)
{
	if (_msg.sysid == 0) return; //this can't be anything meaningful

    if (!isSystemIdValid() || (_msg.sysid != _sysid)) { //we are user friendly and allow the sys_id to change
    	if (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT) {
    		mavlink_heartbeat_t payload;
    		mavlink_msg_heartbeat_decode(&_msg, &payload);
    		if ((_msg.compid == MAV_COMP_ID_AUTOPILOT1) || (payload.autopilot != MAV_AUTOPILOT_INVALID)) {
    			_sysid = _msg.sysid;
    			autopilottype = payload.autopilot;
    			vehicletype = payload.type;
                autopilot.compid = _msg.compid;
    			autopilot.requests_triggered = 1; //we need to postpone it
    		}
    	}
        if (!isSystemIdValid()) return;
    }

    //this is inefficient!! lots of heartbeat decodes

    if ((gimbal.compid == 0) && (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)) {
		mavlink_heartbeat_t payload;
		mavlink_msg_heartbeat_decode(&_msg, &payload);
    	if ( (payload.autopilot == MAV_AUTOPILOT_INVALID) &&
    	     (	(payload.type == MAV_TYPE_GIMBAL) ||
    	    	((_msg.compid == MAV_COMP_ID_GIMBAL) ||
    	    			((_msg.compid >= MAV_COMP_ID_GIMBAL2) && (_msg.compid <= MAV_COMP_ID_GIMBAL6)))  )   ) {
            _resetGimbal();
    		gimbal.compid = _msg.compid;
            gimbal.is_initialized = true; //no startup requests, so true
    	}
    }

    if ((camera.compid == 0) && (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)) {
		mavlink_heartbeat_t payload;
		mavlink_msg_heartbeat_decode(&_msg, &payload);
    	if ( (payload.autopilot == MAV_AUTOPILOT_INVALID) &&
    		 (  (payload.type == MAV_TYPE_CAMERA) ||
    	        ((_msg.compid >= MAV_COMP_ID_CAMERA) && (_msg.compid <= MAV_COMP_ID_CAMERA6))  )  ) {
    	    _resetCamera();
    		camera.compid = _msg.compid;
    		camera.requests_triggered = 1; //we schedule it
    	}
    }

    if (_msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
        _is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT; //reset receiving timeout
    }

    // MAVLINK
    //if (msgFifo_enabled) msgRxFifo.push(_msg);

    // MAVSDK
    // also try to convert the MAVLink messages to FrSky sensors

    // RADIO_STATUS is somewhat tricky, this may need doing it better if there are more sources of it
    // SiK comes as vehicle 51, comp 68!
    // it must NOT be rated as _is_recieving!
	if (_msg.msgid == MAVLINK_MSG_ID_RADIO_STATUS) {
    	mavlink_radio_status_t payload;
    	mavlink_msg_radio_status_decode(&_msg, &payload);
    	radio.rssi = payload.rssi;
    	radio.remrssi = payload.remrssi;
    	radio.noise = payload.noise;
    	radio.remnoise = payload.remnoise;
        radio.is_receiving = MAVLINK_TELEM_RADIO_RECEIVING_TIMEOUT;
        if (g_model.mavlinkMimicSensors) {
            int32_t r = (payload.rssi == UINT8_MAX) ? 0 : payload.rssi;
            setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, RSSI_ID, 0, 1, r, UNIT_DB, 0);
            //#if defined(MULTIMODULE)
            //{ TX_RSSI_ID, TX_RSSI_ID, 0, ZSTR_TX_RSSI   , UNIT_DB , 0 },
            //{ TX_LQI_ID , TX_LQI_ID,  0, ZSTR_TX_QUALITY, UNIT_RAW, 0 },
        }
    	return;
   	}

    // handle messages coming from autopilot
    if (_msg.compid == autopilot.compid) {
    	handleMessageAutopilot();
    }
    if (camera.compid && (_msg.compid == camera.compid)) {
    	handleMessageCamera();
    }
    if (gimbal.compid && (_msg.compid == gimbal.compid)) {
    	handleMessageGimbal();
    }
}


// -- Wakeup call from OpenTx --
// this is the single and main entry point
void MavlinkTelem::wakeup()
{
#if defined(MAVLINK_TELEM)
    // TODO: we want to have a configuration enum
    if ((_interface_enabled != g_model.mavlinkEnabled) || (_interface_config != g_model.mavlinkConfig)) { // a change occurred
        mavlinkTelemDeInit();
        _interface_enabled = g_model.mavlinkEnabled;
        _interface_config = g_model.mavlinkConfig;
        if (_interface_enabled) {
            switch (_interface_config) {
            case CONFIG_UART_A_115200: mavlinkTelemInit('A', 57600); break;
            case CONFIG_UART_A_57600: mavlinkTelemInit('A', 115200); break;
            case CONFIG_UART_A_38400: mavlinkTelemInit('A', 38400); break;
            case CONFIG_UART_A_19200: mavlinkTelemInit('A', 19200); break;
            default: mavlinkTelemDeInit(); // should never happen
            }
        }
    }
    if (!_interface_enabled) return;

    // look for incoming messages, also do statistics
	uint32_t available = mavlinkTelemAvailable();
	if (available > 128) available = 128; //limit how much we read at once, shouldn't ever trigger
	for (uint32_t i = 0; i < available; i++){
		uint8_t c;
		mavlinkTelemGetc(&c);
		_bytes_rx_persec_cnt++;
	    if (mavlink_parse_char(MAVLINK_COMM_0, c, &_msg, &_status)) {
	    	// check for lost messages by analyzing seq
	        if (_seq_rx_last >= 0) {
	            uint16_t seq = _msg.seq;
	        	if (seq < _seq_rx_last) seq += 256;
	        	_seq_rx_last++;
	            if (seq > _seq_rx_last) msg_rx_lost += (seq - _seq_rx_last);
	        }
	        _seq_rx_last = _msg.seq;
        	handleMessage();
			msg_rx_count++;
			_msg_rx_persec_cnt++;
			if (g_model.mavlinkMimicSensors) telemetryStreaming = 2*TELEMETRY_TIMEOUT10ms; // 2 seconds
	    }
	}

	// receiving timeouts
	if (_is_receiving) {
		_is_receiving--;
		if (!_is_receiving) _reset(); //this also resets is_receiving of all other components
	}
	if (radio.is_receiving) {
	    radio.is_receiving--;
	    if (!radio.is_receiving) _resetRadio();
	}
	if (autopilot.is_receiving) {
		autopilot.is_receiving--;
		if (!autopilot.is_receiving) _resetAutopilot();
	}
	if (gimbal.is_receiving) {
		gimbal.is_receiving--;
		if (!gimbal.is_receiving) _resetGimbal();
	}
	if (camera.is_receiving) {
		camera.is_receiving--;
		if (!camera.is_receiving) _resetCamera();
	}

    // do tasks
	doTask();

    // send out any pending messages
	if (_txcount) {
		if (mavlinkTelemPutBuf(_txbuf, _txcount)) {
            _txcount = 0;
		}
	}

#endif
}


// -- Resets --

void MavlinkTelem::_resetAutopilot(void)
{
    _task[TASK_AUTOPILOT] = 0;
    _task[TASK_AP] = 0;

    autopilot.compid = 0;
    autopilot.is_receiving = 0;
    autopilot.requests_triggered = 0;
    autopilot.requests_waiting_mask = AUTOPILOT_REQUESTWAITING_ALL;
    autopilot.is_initialized = false;

    autopilot.system_status = MAV_STATE_UNINIT;
	autopilot.custom_mode = 0;
    autopilot.is_armed = false;
    autopilot.is_standby = true;
    autopilot.is_critical = false;
    autopilot.prearm_ok = false;
    autopilot.updated = 0;

	att.roll_rad = 0.0f;
	att.pitch_rad = 0.0f;
	att.yaw_rad = 0.0f;
    att.updated = 0;

    gps1.fix = GPS_FIX_TYPE_NO_GPS;
    gps1.sat = UINT8_MAX;
    gps1.hdop = UINT16_MAX;
    gps1.vdop = UINT16_MAX;
    gps1.lat = 0;
    gps1.lon = 0;
    gps1.alt_mm = 0;
    gps1.vel_cmps = UINT16_MAX;
    gps1.cog_cdeg = UINT16_MAX;
    gps1.updated = 0;

    gps2.fix = GPS_FIX_TYPE_NO_GPS;
    gps2.sat = UINT8_MAX;
    gps2.hdop = UINT16_MAX;
    gps2.vdop = UINT16_MAX;
    gps2.lat = 0;
    gps2.lon = 0;
    gps2.alt_mm = 0;
    gps2.vel_cmps = UINT16_MAX;
    gps2.cog_cdeg = UINT16_MAX;
    gps2.updated = 0;

    gps_instancemask = 0;

    gposition.lat = 0;
    gposition.lon = 0;
    gposition.alt_mm = 0;
    gposition.relative_alt_mm = 0;
    gposition.vx_cmps = 0;
    gposition.vy_cmps = 0;
    gposition.vz_cmps = 0;
    gposition.hdg_cdeg = UINT16_MAX;
    gposition.updated = 0;

	vfr.airspd_mps = 0.0f;
	vfr.groundspd_mps = 0.0f;
	vfr.alt_m = 0.0f;
	vfr.climbrate_mps = 0.0f;
	vfr.heading_deg = 0;
	vfr.thro_pct = 0;
    vfr.updated = 0;

	bat1.charge_consumed_mAh = -1;
	bat1.energy_consumed_hJ = -1;
	bat1.temperature_cC = INT16_MAX;
	bat1.voltage_mV = 0;
	bat1.current_cA = -1;
	bat1.remaining_pct = -1;
	bat1.cellcount = -1;
    bat1.updated = 0;

	bat2.charge_consumed_mAh = -1;
	bat2.energy_consumed_hJ = -1;
	bat2.temperature_cC = INT16_MAX;
	bat2.voltage_mV = 0;
	bat2.current_cA = -1;
	bat2.remaining_pct = -1;
	bat2.cellcount = -1;
    bat2.updated = 0;

    bat_instancemask = 0;

    statustext.fifo.clear();
    statustext.updated =  0;

    ekf.flags = 0;
    ekf.updated = 0;
}


void MavlinkTelem::_resetGimbal(void)
{
    _task[TASK_GIMBAL] = 0;

	gimbal.compid = 0;
	gimbal.is_receiving = 0;
    gimbal.requests_triggered = 0;
    gimbal.requests_waiting_mask = 0;
    gimbal.is_initialized = false;

	gimbal.system_status = MAV_STATE_UNINIT;
	gimbal.custom_mode = 0;
	gimbal.is_armed = false;
	gimbal.is_standby = true;
	gimbal.is_critical = false;
	gimbal.prearm_ok = false;
	gimbal.updated = 0;

    gimbalAtt.roll_deg = 0.0f;
    gimbalAtt.pitch_deg = 0.0f;
    gimbalAtt.yaw_deg_relative = 0.0f;
    gimbalAtt.yaw_deg_absolute = 0.0f;
    gimbalAtt.updated = 0;
}


void MavlinkTelem::_resetCamera(void)
{
    _task[TASK_CAMERA] = 0;

    camera.compid = 0;
    camera.is_receiving = 0;
    camera.requests_triggered = 0;
    camera.requests_waiting_mask = CAMERA_REQUESTWAITING_ALL;
    camera.is_initialized = false;

    camera.system_status = MAV_STATE_UNINIT;
	camera.custom_mode = 0;
    camera.is_armed = false;
    camera.is_standby = true;
    camera.is_critical = false;
    camera.prearm_ok = false;
    camera.updated = 0;

	cameraInfo.vendor_name[0] = 0;
	cameraInfo.model_name[0] = 0;
	cameraInfo.flags = 0;
	cameraInfo.has_video = false;
	cameraInfo.has_photo = false;
	cameraInfo.has_modes = false;
	cameraInfo.total_capacity_MiB = NAN;

	cameraStatus.mode = 0;
	cameraStatus.video_on = false;
	cameraStatus.photo_on = false;
	cameraStatus.available_capacity_MiB = NAN;
	cameraStatus.recording_time_ms = UINT32_MAX;
	cameraStatus.battery_voltage_V = NAN;
	cameraStatus.battery_remaining_pct = -1;
}


void MavlinkTelem::_resetRadio(void)
{
    radio.is_receiving = 0;

    radio.rssi = UINT8_MAX;
    radio.remrssi = UINT8_MAX;
    radio.noise = 0;
    radio.remnoise = 0;
}


void MavlinkTelem::_reset(void)
{
	mavlink_reset_channel_status(MAVLINK_COMM_0);

    _my_sysid = MAVLINK_TELEM_MY_SYSID;
    _my_compid = MAVLINK_TELEM_MY_COMPID;

	msg_rx_count = msg_rx_lost = 0;
	msg_rx_persec = bytes_rx_persec = 0;
    _msg_rx_persec_cnt = _bytes_rx_persec_cnt = 0;
    _seq_rx_last = -1;

    _sysid = 0;
    autopilottype = MAV_AUTOPILOT_GENERIC; //TODO: shouldn't these be in _resetAutopilot() ??
	vehicletype = MAV_TYPE_GENERIC;
	flightmode = 0;

	for (uint16_t i = 0; i < TASKIDX_MAX; i++) _task[i] = 0;
    _taskFifo.clear();
    _taskFifo_tlast = 0;
    for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) _requestList[i].task = 0;

    _resetRadio();
	_resetAutopilot();
	_resetGimbal();
	_resetCamera();

	// MAVLINK
	msgRxFifo.clear();
	msgFifo_enabled = false;
}


// -- Miscellaneous --

// ArduPilot starts with sending heartbeat every 1 sec with FE, TimeSync every 5 sec with FE
// we then need to request the data stream
// TODO: we can also request them individually now
void MavlinkTelem::requestDataStreamFromAutopilot(void)
{
    if (autopilottype == MAV_AUTOPILOT_ARDUPILOTMEGA) {
        // 2Hz sufficient, cleared by MAVLINK_MSG_ID_GPS_RAW_INT
        // yields: MAVLINK_MSG_ID_GPS_RAW_INT
        //         MAVLINK_MSG_ID_GPS2_RAW
        //         MAVLINK_MSG_ID_SYS_STATUS
        set_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTENDED_STATUS, 100, 196);

        // cleared by MAVLINK_MSG_ID_GLOBAL_POSITION_INT
        // yields: MAVLINK_MSG_ID_GLOBAL_POSITION_INT
        //set_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_POSITION, 100, 198);

        // cleared by MAVLINK_MSG_ID_ATTITUDE
        // yields: MAVLINK_MSG_ID_ATTITUDE
        //set_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA1, 100, 211);

        // 2Hz sufficient, cleared by MAVLINK_MSG_ID_VFR_HUD
        // yields: MAVLINK_MSG_ID_VFR_HUD
        set_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA2, 100, 220);

        // 2Hz sufficient, cleared by MAVLINK_MSG_ID_BATTERY_STATUS
        // yields: MAVLINK_MSG_ID_BATTERY_STATUS
        //         MAVLINK_MSG_ID_EKF_STATUS_REPORT
        set_request(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA3, 100, 203);

        // call these at high rates of 10 Hz
        set_request(TASK_AUTOPILOT, TASK_SENDCMD_REQUEST_ATTITUDE, 100, 205);
        set_request(TASK_AUTOPILOT, TASK_SENDCMD_REQUEST_GLOBAL_POSITION_INT, 100, 207);

        push_task(TASK_AP, TASK_ARDUPILOT_REQUESTBANNER);
        return;
    }
    // other autopilots
    // TODO
}






/*
ArduPilot Copter Streams:

static const ap_message STREAM_RAW_SENSORS_msgs[] = {
    MSG_RAW_IMU,
    MSG_SCALED_IMU2,
    MSG_SCALED_IMU3,
    MSG_SCALED_PRESSURE,
    MSG_SCALED_PRESSURE2,
    MSG_SCALED_PRESSURE3,
    MSG_SENSOR_OFFSETS
};
static const ap_message STREAM_EXTENDED_STATUS_msgs[] = {
    MSG_SYS_STATUS,
    MSG_POWER_STATUS,
    MSG_MEMINFO,
    MSG_CURRENT_WAYPOINT, // MISSION_CURRENT
    MSG_GPS_RAW,
    MSG_GPS_RTK,
    MSG_GPS2_RAW,
    MSG_GPS2_RTK,
    MSG_NAV_CONTROLLER_OUTPUT,
    MSG_FENCE_STATUS,
    MSG_POSITION_TARGET_GLOBAL_INT,
};
static const ap_message STREAM_POSITION_msgs[] = {
    MSG_LOCATION,
    MSG_LOCAL_POSITION
};
static const ap_message STREAM_RC_CHANNELS_msgs[] = {
    MSG_SERVO_OUTPUT_RAW,
    MSG_RC_CHANNELS,
    MSG_RC_CHANNELS_RAW, // only sent on a mavlink1 connection
};
static const ap_message STREAM_EXTRA1_msgs[] = {
    MSG_ATTITUDE,
    MSG_SIMSTATE,
    MSG_AHRS2,
    MSG_AHRS3,
    MSG_PID_TUNING // Up to four PID_TUNING messages are sent, depending on GCS_PID_MASK parameter
};
static const ap_message STREAM_EXTRA2_msgs[] = {
    MSG_VFR_HUD
};
static const ap_message STREAM_EXTRA3_msgs[] = {
    MSG_AHRS,
    MSG_HWSTATUS,
    MSG_SYSTEM_TIME,
    MSG_RANGEFINDER,
    MSG_DISTANCE_SENSOR,
#if AP_TERRAIN_AVAILABLE && AC_TERRAIN
    MSG_TERRAIN,
#endif
    MSG_BATTERY2,
    MSG_BATTERY_STATUS,
    MSG_MOUNT_STATUS,
    MSG_OPTICAL_FLOW,
    MSG_GIMBAL_REPORT,
    MSG_MAG_CAL_REPORT,
    MSG_MAG_CAL_PROGRESS,
    MSG_EKF_STATUS_REPORT,
    MSG_VIBRATION,
    MSG_RPM,
    MSG_ESC_TELEMETRY,
};
static const ap_message STREAM_PARAMS_msgs[] = {
    MSG_NEXT_PARAM
};
static const ap_message STREAM_ADSB_msgs[] = {
    MSG_ADSB_VEHICLE
};
*/

/*
AP battery vs sys_status

=> BATTERY sends the very same data as SYS_STATUS, but SYS_STATUS has initialized() and health() tests in addition
=> not clear which is better
let's decide to use BATTERY

BATTERY:
    AP_BattMonitor::cells fake_cells;
    sends total voltage splited up into cells of 65534

    float current, consumed_mah, consumed_wh;
    if (battery.current_amps(current, instance)) {
         current *= 100;
    } else {
        current = -1;
    }
    if (!battery.consumed_mah(consumed_mah, instance)) {
        consumed_mah = -1;
    }
    if (battery.consumed_wh(consumed_wh, instance)) {
        consumed_wh *= 36;
    } else {
        consumed_wh = -1;
    }

    mavlink_msg_battery_status_send(chan,
                                    instance, // id
                                    MAV_BATTERY_FUNCTION_UNKNOWN, // function
                                    MAV_BATTERY_TYPE_UNKNOWN, // type
                                    got_temperature ? ((int16_t) (temp * 100)) : INT16_MAX, // temperature. INT16_MAX if unknown
                                    battery.has_cell_voltages(instance) ? battery.get_cell_voltages(instance).cells : fake_cells.cells, // cell voltages
                                    current,      // current in centiampere
                                    consumed_mah, // total consumed current in milliampere.hour
                                    consumed_wh,  // consumed energy in hJ (hecto-Joules)
                                    battery.capacity_remaining_pct(instance),
                                    0, // time remaining, seconds (not provided)
                                    MAV_BATTERY_CHARGE_STATE_UNDEFINED);

SYS_STATUS:
    if (!gcs().vehicle_initialised()) {
        return;
    }

    const AP_BattMonitor &battery = AP::battery();
    float battery_current;
    int8_t battery_remaining;

    if (battery.healthy() && battery.current_amps(battery_current)) {
        battery_remaining = battery.capacity_remaining_pct();
        battery_current *= 100;
    } else {
        battery_current = -1;
        battery_remaining = -1;
    }

    ...

    mavlink_msg_sys_status_send(
        chan,
        control_sensors_present,
        control_sensors_enabled,
        control_sensors_health,
        static_cast<uint16_t>(AP::scheduler().load_average() * 1000),
        battery.voltage() * 1000,  // mV
        battery_current,        // in 10mA units
        battery_remaining,      // in %
        0,  // comm drops %,
        0,  // comm drops in pkts,
        errors1,
        errors2,
        0,  // errors3
        errors4); // errors4
*/

/*
DATA_ID == 0x5006 then -- ROLLPITCH
DATA_ID == 0x5005 then -- VELANDYAW
DATA_ID == 0x5001 then -- AP STATUS
DATA_ID == 0x5002 then -- GPS STATUS
DATA_ID == 0x5003 then -- BATT
DATA_ID == 0x5008 then -- BATT2
DATA_ID == 0x5004 then -- HOME
DATA_ID == 0x5000 then -- MESSAGES
DATA_ID == 0x5007 then -- PARAMS
DATA_ID == 0x5009 then -- WAYPOINTS @1Hz
DATA_ID == 0x50F1 then -- RC CHANNELS
DATA_ID == 0x50F2 then -- VFR

AP
DATA_ID == 0x5000 then -- MESSAGES
DATA_ID == 0x5006 then -- ROLLPITCH
x0800   GPS LAT LON
DATA_ID == 0x5005 then -- VELANDYAW
DATA_ID == 0x5001 then -- AP STATUS
DATA_ID == 0x5002 then -- GPS STATUS
DATA_ID == 0x5004 then -- HOME
DATA_ID == 0x5008 then -- BATT2
DATA_ID == 0x5003 then -- BATT
DATA_ID == 0x5007 then -- PARAMS
 */
/*
	int32_t data = 12345 + _data;
	_data++;
#if defined(LUA)
    if (luaInputTelemetryFifo && luaInputTelemetryFifo->hasSpace(sizeof(SportTelemetryPacket))) {
      SportTelemetryPacket luaPacket;
      luaPacket.physicalId = 0; //sensor_id, isn't used
      luaPacket.primId = 0x10; //frame_id, only 0x10 is evaluated
      luaPacket.dataId = 0x5003; //data_id:
      luaPacket.value = data; //value
      for (uint8_t i=0; i<sizeof(SportTelemetryPacket); i++) {
        luaInputTelemetryFifo->push(luaPacket.raw[i]);
      }
    }
#endif
 */


/*
DroneKit
is_armable(self):
    return self.mode != 'INITIALISING' and (self.gps_0.fix_type is not None and self.gps_0.fix_type > 1) and self._ekf_predposhorizabs
ekf_ok(self):
    # use same check that ArduCopter::system.pde::position_ok() is using
    if self.armed: return self._ekf_poshorizabs and not self._ekf_constposmode
    else: return self._ekf_poshorizabs or self._ekf_predposhorizabs
groundspeed(self, speed):
    command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, speed, -1, 0, 0, 0, 0)
airspeed(self, speed):
    command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED,  0, 0, speed, -1, 0, 0, 0, 0)
simple_takeoff(self, alt=None):
    command_long_send(0, 0, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, altitude)
simple_goto(self, location, airspeed=None, groundspeed=None):
    frame = mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT
    mission_item_send(0, 0, 0, frame, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 2, 0, 0, 0, 0, 0, location.lat, location.lon, alt)

gimbal
rotate(self, pitch, roll, yaw):
    mount_configure_encode( 0, 1, mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING, 1, 1, 1)
    mount_control_encode(0, 1, pitch * 100, roll * 100, yaw * 100, 0)
target_location(self, roi):
    mount_configure_encode(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_GPS_POINT, 1, 1, 1)
    command_long_encode(0, 1, mavutil.mavlink.MAV_CMD_DO_SET_ROI, 0, 0, 0, 0, 0, roi.lat, roi.lon, alt)
release(self):
    mount_configure_encode(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_RC_TARGETING, 1, 1, 1)


solo shots
class VectorPathHandler(PathHandler):
def move(self, channels):
def travel(self):
    set_position_target_global_int_encode(
            0, 0, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b0000110111000000,
            int(loc.lat * 10000000), int(loc.lon * 10000000), loc.alt,
            velVector.x, velVector.y, velVector.z,
            0, 0, 0,
            0, 0)

 initStreamRates(self):
        STREAM_RATES = {
            mavutil.mavlink.MAV_DATA_STREAM_EXTENDED_STATUS: 2,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA1: 10,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA2: 10,
            mavutil.mavlink.MAV_DATA_STREAM_EXTRA3: 2,
            mavutil.mavlink.MAV_DATA_STREAM_POSITION: 10,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_SENSORS: 2,
            mavutil.mavlink.MAV_DATA_STREAM_RAW_CONTROLLER: 3,
            mavutil.mavlink.MAV_DATA_STREAM_RC_CHANNELS: 5,
        }

class TwoPointPathHandler(PathHandler):
def MoveTowardsEndpt( self, channels ):
    simple_goto(target)
    command_long_encode(0, 1, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, abs(self.currentSpeed), -1, 0.0, 0.0, 0.0, 0.0)

class MultipointShot():
def handleRCs(self, channels):
    set_position_target_global_int_encode(
            0, 0, 1, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,
            0b0000110111000000,
            int(self.commandPos.lat * 10000000), int(self.commandPos.lon * 10000000),
            self.commandPos.alt,
            self.commandVel.x, self.commandVel.y, self.commandVel.z,
            0, 0, 0,
            0, 0)
def enterRecordMode(self):
        mode = VehicleMode("LOITER")
def enterPlayMode(self):
        mode = VehicleMode("GUIDED")
        mount_configure_encode(0, 1, mavutil.mavlink.MAV_MOUNT_MODE_MAVLINK_TARGETING, 1, 1, 1)
def handleAttach(self, attach):
        simple_goto(self.commandPos)
        command_long_encode(0, 1, mavutil.mavlink.MAV_CMD_DO_CHANGE_SPEED, 0, 1, ATTACH_SPEED, -1, 0.0, 0.0, 0.0, 0.0)


 */
