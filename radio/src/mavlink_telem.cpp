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


// -- TASK handlers --

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

    _request_waiting[idx] |= task;

    if (retry == 0) return; //well, if there would be another pending we would not kill it

    int8_t empty_i = -1;

    // first check if request is already pending, at the same time find free slot, to avoid having to loop twice
    for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
        // TODO: should we modify the retry & rate of the pending task?
        if ((_requestList[i].idx == idx) && (_requestList[i].task == task)) return; // already pending, we can get out of here
        if ((empty_i < 0) && !_requestList[i].task) empty_i = i;
    }

    // if not already pending, add it
    if (empty_i < 0) return; // no free slot

    _requestList[empty_i].task = task;
    _requestList[empty_i].idx = idx;
    _requestList[empty_i].retry = retry;
    _requestList[empty_i].tlast = get_tmr10ms();
    _requestList[empty_i].trate = rate; // every ca 1 sec
}


void MavlinkTelem::clear_request(uint8_t idx, uint32_t task)
{
    for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
        if ((_requestList[i].idx == idx) && (_requestList[i].task == task)) {
            _requestList[i].task = 0;
            _request_waiting[idx] &=~ task;
        }
    }
}

//TODO: what happens if a clear never comes?

void MavlinkTelem::do_requests(void)
{
    tmr10ms_t tnow = get_tmr10ms();

    for (uint16_t i = 0; i < TASKIDX_MAX; i++) _request_waiting[i] = 0;

    for (uint16_t i = 0; i < REQUESTLIST_MAX; i++) {
        if (!_requestList[i].task) continue;

        _request_waiting[_requestList[i].idx] |= _requestList[i].task;

        if ((tnow - _requestList[i].tlast) >= _requestList[i].trate) {
            push_task(_requestList[i].idx, _requestList[i].task);
            _requestList[i].tlast = get_tmr10ms();
            if (_requestList[i].retry < UINT8_MAX) {
                if (_requestList[i].retry) _requestList[i].retry--;
                if (!_requestList[i].retry) _requestList[i].task = 0;
            }
        }
    }

    if ((tnow - _taskFifo_tlast) > 11) { // 110 ms decimation
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

void MavlinkTelem::generateHeartbeat(uint8_t base_mode, uint32_t custom_mode, uint8_t system_status)
{
	setOutVersionV2();
	mavlink_msg_heartbeat_pack(
		  _my_sysid, _my_compid, &_msg_out,
          MAV_TYPE_GCS, MAV_AUTOPILOT_INVALID, base_mode, custom_mode, system_status
		  //MAV_TYPE_GIMBAL, MAV_AUTOPILOT_INVALID, base_mode, custom_mode, system_status
		  );
	_txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}


void MavlinkTelem::generateRequestDataStream(uint8_t tsystem, uint8_t tcomponent, uint8_t data_stream, uint16_t rate, uint8_t startstop)
{
	setOutVersionV2();
	mavlink_msg_request_data_stream_pack(
			_my_sysid, _my_compid, &_msg_out,
			tsystem, tcomponent, data_stream, rate, startstop
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

void MavlinkTelem::_generateCmdLong(uint8_t tsystem, uint8_t tcomponent, uint16_t cmd, float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    setOutVersionV2();
    mavlink_msg_command_long_pack(
            _my_sysid, _my_compid, &_msg_out,
            tsystem, tcomponent, cmd, 0, p1, p2, p3, p4, p5, p6, p7
            );
    _txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}

//for ArduPilot:
//  base_mode must have MAV_MODE_FLAG_CUSTOM_MODE_ENABLED bit set,
//  custom_mode then determines the mode it will switch to
//  usage of this cmd is thus very likely very flightstack dependent!!
void MavlinkTelem::generateCmdDoSetMode(uint8_t tsystem, uint8_t tcomponent, MAV_MODE base_mode, uint32_t custom_mode)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_DO_SET_MODE, base_mode, custom_mode);
}

//for ArduPilot:
//  doesn't support acceleration
//  position:  type_mask = 0x0DF8
//  position & velocity: type_mask = 0x0DC0
//  velocity: type_mask = 0x0DC7
void MavlinkTelem::generateSetPositionTargetGlobalInt(uint8_t tsystem, uint8_t tcomponent, uint8_t coordinate_frame, uint16_t type_mask, int32_t lat, int32_t lon, float alt, float vx, float vy, float vz, float yaw, float yaw_rate)
{
    setOutVersionV2();
    mavlink_msg_set_position_target_global_int_pack(
            _my_sysid, _my_compid, &_msg_out,
            0, //uint32_t time_boot_ms
            tsystem, tcomponent,
            coordinate_frame, type_mask,
            lat, lon, alt, vx, vy, vz, 0.0f, 0.0f, 0.0f, yaw, yaw_rate
            );
    _txcount = mavlink_msg_to_send_buffer(_txbuf, &_msg_out);
}


void MavlinkTelem::generateRequestCameraInformation(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_REQUEST_CAMERA_INFORMATION, 1, 0,0,0,0,0,0);
}

void MavlinkTelem::generateRequestCameraSettings(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_REQUEST_CAMERA_SETTINGS, 1, 0,0,0,0,0,0);
}

void MavlinkTelem::generateRequestStorageInformation(uint8_t tsystem, uint8_t tcomponent)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_REQUEST_STORAGE_INFORMATION, 0, 1, 0,0,0,0,0);
}

void MavlinkTelem::generateRequestCameraCapturesStatus(uint8_t tsystem, uint8_t tcomponent)
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

void MavlinkTelem::generateCmdDoMountControl(uint8_t tsystem, uint8_t tcomponent, float pitch, float yaw)
{
    _generateCmdLong(tsystem, tcomponent, MAV_CMD_DO_MOUNT_CONTROL, pitch, 0.0, yaw, 0,0,0, MAV_MOUNT_MODE_MAVLINK_TARGETING);
}


// -- Mavsdk Convenience Wrapper --

void MavlinkTelem::apSetFlightMode(uint32_t ap_flight_mode)
{
    _t_base_mode = MAV_MODE_FLAG_CUSTOM_MODE_ENABLED;
    _t_custom_mode = ap_flight_mode;
    SETTASK(TASK_AUTOPILOT, TASK_SENDCMD_SET_MODE);
}

//note, we can enter negative yaw here, sign determines direction
void MavlinkTelem::apSetYaw(float yaw, bool relative)
{
    if (relative) {
        _tsy_yaw_relative = 1.0f;
        if (yaw < 0.0f){ _tsy_yaw_dir = -1.0f; yaw = -yaw; } else{ _tsy_yaw_dir = 1.0f; }
    } else {
        _tsy_yaw_relative = 0.0f;
        _tsy_yaw_dir = 0.0f;
    }
    float res = fmodf(yaw, 360.0f); //yaw must be in range [0..360]
    if (res < 0.0f) res += 360.0f;
    _tsy_yaw = res;
    SETTASK(TASK_AUTOPILOT, TASK_SENDCMD_CONDITION_YAW);
}

void MavlinkTelem::apGotoPositionAltYaw(int32_t lat, int32_t lon, float alt, float yaw)
{
    _t_coordinate_frame = MAV_FRAME_GLOBAL_RELATIVE_ALT_INT;
    //_t_type_mask = 0x0DF8;
    _t_type_mask = POSITION_TARGET_TYPEMASK_VX_IGNORE | POSITION_TARGET_TYPEMASK_VY_IGNORE | POSITION_TARGET_TYPEMASK_VZ_IGNORE |
                   POSITION_TARGET_TYPEMASK_AX_IGNORE | POSITION_TARGET_TYPEMASK_AY_IGNORE | POSITION_TARGET_TYPEMASK_AZ_IGNORE |
                   POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE;
    if (isnan(alt)) _t_type_mask |= POSITION_TARGET_TYPEMASK_Z_IGNORE;
    if (isnan(yaw)) _t_type_mask |= POSITION_TARGET_TYPEMASK_YAW_IGNORE;
    _t_lat = lat; _t_lon = lon; _t_alt = alt; _t_vx = _t_vy = _t_vz = 0.0f; _t_yaw = yaw; _t_yaw_rate = 0.0f;
    SETTASK(TASK_AUTOPILOT, TASK_SENDMSG_SET_POSITION_TARGET_GLOBAL_INT);
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

    if (!cameraStatus.initialized) {
    	cameraStatus.initialized = (cameraInfo.info_received && cameraInfo.settings_received && cameraInfo.status_received);
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
        if (_task[TASK_AUTOPILOT] & TASK_SENDCMD_SET_MODE) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDCMD_SET_MODE);
            generateCmdDoSetMode(_sysid, autopilot.compid, (MAV_MODE)_t_base_mode, _t_custom_mode);
            return; //do only one per loop
        }
        if (_task[TASK_AUTOPILOT] & TASK_SENDMSG_SET_POSITION_TARGET_GLOBAL_INT) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDMSG_SET_POSITION_TARGET_GLOBAL_INT);
            generateSetPositionTargetGlobalInt(_sysid, autopilot.compid, _t_coordinate_frame, _t_type_mask, _t_lat, _t_lon, _t_alt, _t_vx, _t_vy, _t_vz, _t_yaw, _t_yaw_rate);
            return; //do only one per loop
        }
        if (_task[TASK_AUTOPILOT] & TASK_SENDCMD_CONDITION_YAW) {
            RESETTASK(TASK_AUTOPILOT,TASK_SENDCMD_CONDITION_YAW);
            _generateCmdLong(_sysid, autopilot.compid, _tsy_yaw, 0.0f, _tsy_yaw_dir, _tsy_yaw_relative);
            return; //do only one per loop
        }

		if (_task[TASK_AUTOPILOT] & TASK_SENDMSG_PARAM_REQUEST_LIST) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDMSG_PARAM_REQUEST_LIST);
	        generateParamRequestList(_sysid, autopilot.compid);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_RAW_SENSORS) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_RAW_SENSORS);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_RAW_SENSORS, 2, 1);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_EXTENDED_STATUS) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_EXTENDED_STATUS);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_EXTENDED_STATUS, 2, 1);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_POSITION) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_POSITION);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_POSITION, 2, 1);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_EXTRA1) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_EXTRA1);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_EXTRA1, 4, 1);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_EXTRA2) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_EXTRA2);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_EXTRA2, 4, 1);
	        return; //do only one per loop
		}
		if (_task[TASK_AUTOPILOT] & TASK_SENDREQUESTDATASTREAM_EXTRA3) {
	        RESETTASK(TASK_AUTOPILOT,TASK_SENDREQUESTDATASTREAM_EXTRA3);
	        generateRequestDataStream(_sysid, autopilot.compid, MAV_DATA_STREAM_EXTRA3, 2, 1);
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
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_NAV_TAKEOFF, 0,0, 0.0f, 0,0,0, _t_takeoff_alt); //must_navigate = true
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
            _generateCmdLong(_sysid, autopilot.compid, MAV_CMD_SOLO_BTN_FLY_HOLD, _t_takeoff_alt);
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
	        if (camera.compid) generateCmdSetCameraMode(_sysid, camera.compid, CAMERA_MODE_VIDEO);
	        set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS, 2);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDCMD_SET_CAMERA_PHOTO_MODE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_SET_CAMERA_PHOTO_MODE);
	        if (camera.compid) generateCmdSetCameraMode(_sysid, camera.compid, CAMERA_MODE_IMAGE);
            set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS, 2);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDCMD_IMAGE_START_CAPTURE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_IMAGE_START_CAPTURE);
	        if (camera.compid) generateCmdImageStartCapture(_sysid, camera.compid);
            set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS, 2);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDCMD_VIDEO_START_CAPTURE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_VIDEO_START_CAPTURE);
	        if (camera.compid) generateCmdVideoStartCapture(_sysid, camera.compid);
            set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS, 2);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDCMD_VIDEO_STOP_CAPTURE) {
	        RESETTASK(TASK_CAMERA, TASK_SENDCMD_VIDEO_STOP_CAPTURE);
	        if (camera.compid) generateCmdVideoStopCapture(_sysid, camera.compid);
            set_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS, 2);
	        return; //do only one per loop
		}

		// the sequence here defines the startup sequence
		if (_task[TASK_CAMERA] & TASK_SENDREQUEST_CAMERA_INFORMATION) {
	        RESETTASK(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_INFORMATION);
	        if (camera.compid) generateRequestCameraInformation(_sysid, camera.compid);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDREQUEST_CAMERA_SETTINGS) {
	        RESETTASK(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS);
	        if (camera.compid) generateRequestCameraSettings(_sysid, camera.compid);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS) {
	        RESETTASK(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS);
	        if (camera.compid) generateRequestCameraCapturesStatus(_sysid, camera.compid);
	        return; //do only one per loop
		}
		if (_task[TASK_CAMERA] & TASK_SENDREQUEST_STORAGE_INFORMATION) {
	        RESETTASK(TASK_CAMERA, TASK_SENDREQUEST_STORAGE_INFORMATION);
	        if (camera.compid) generateRequestStorageInformation(_sysid, camera.compid);
	        return; //do only one per loop
		}

	    // gimbal tasks
		if (_task[TASK_GIMBAL] & TASK_SENDCMD_DO_MOUNT_CONFIGURE) {
	        RESETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_MOUNT_CONFIGURE);
	        generateCmdDoMountConfigure(_sysid, autopilot.compid, _t_gimbal_domountconfigure_mode);
	        return; //do only one per loop
		}
		if (_task[TASK_GIMBAL] & TASK_SENDCMD_DO_MOUNT_CONTROL) {
	        RESETTASK(TASK_GIMBAL, TASK_SENDCMD_DO_MOUNT_CONTROL);
	        generateCmdDoMountControl(_sysid, autopilot.compid, _t_gimbal_domountcontrol_pitch, _t_gimbal_domountcontrol_yaw);
	        return; //do only one per loop
		}
	}
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
        cameraInfo.info_received = true;
        clear_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_INFORMATION);
		}break;

	case MAVLINK_MSG_ID_CAMERA_SETTINGS: {
		mavlink_camera_settings_t payload;
		mavlink_msg_camera_settings_decode(&_msg, &payload);
		cameraStatus.mode = (payload.mode_id == CAMERA_MODE_IMAGE) ? CAMERA_MODE_IMAGE : CAMERA_MODE_VIDEO;
        cameraInfo.settings_received = true;
        clear_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_SETTINGS);
		}break;

	case MAVLINK_MSG_ID_CAMERA_CAPTURE_STATUS: {
		mavlink_camera_capture_status_t payload;
		mavlink_msg_camera_capture_status_decode(&_msg, &payload);
		cameraStatus.recording_time_ms = payload.recording_time_ms;
		cameraStatus.available_capacity = payload.available_capacity;
		cameraStatus.video_on = (payload.video_status > 0);
		cameraStatus.photo_on = (payload.image_status > 0); //0: idle, 1: capture in progress, 2: interval set but idle, 3: interval set and capture in progress
        cameraInfo.status_received = true;
        clear_request(TASK_CAMERA, TASK_SENDREQUEST_CAMERA_CAPTURE_STATUS);
		}break;

	case MAVLINK_MSG_ID_STORAGE_INFORMATION: {
		mavlink_storage_information_t payload;
		mavlink_msg_storage_information_decode(&_msg, &payload);
        if (payload.status == STORAGE_STATUS_READY) {
			cameraInfo.total_capacity = payload.total_capacity;
			cameraStatus.available_capacity = payload.available_capacity;
		} else {
			cameraInfo.total_capacity = NAN;
			cameraStatus.available_capacity = NAN;
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
  		cameraStatus.battery_voltage = (has_voltage) ? 0.001f * (float)voltage : NAN;
    	cameraStatus.battery_remainingpct = payload.battery_remaining; // -1 if not known
		}break;

	}
}


// -- Handle incoming MAVLink messages, which are for the Gimbal --

void MavlinkTelem::handleMessageGimbal(void)
{
	switch (_msg.msgid) {

	#define FPI 3.141592654f
	#define FDEGTORAD (FPI/180.0f)

	case MAVLINK_MSG_ID_HEARTBEAT: {
		mavlink_heartbeat_t payload;
		mavlink_msg_heartbeat_decode(&_msg, &payload);
		gimbal.system_status = payload.system_status;
		gimbal.custom_mode = payload.custom_mode;
		gimbal.is_armed = (payload.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) ? true : false;
        gimbal.is_standby = (payload.system_status <= MAV_STATE_STANDBY) ? true : false;
        gimbal.is_critical = (payload.system_status >= MAV_STATE_CRITICAL) ? true : false;
		gimbal.prearm_ok = (payload.custom_mode & 0x80000000) ? false : true;
        gimbal.is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT;
		}break;

    case MAVLINK_MSG_ID_ATTITUDE: {
    	mavlink_attitude_t payload;
        mavlink_msg_attitude_decode(&_msg, &payload);
        gimbalAtt.roll = payload.roll;
        gimbalAtt.pitch = payload.pitch;
        gimbalAtt.yaw_relative = payload.yaw;
        gimbalAtt.yaw_absolute = gimbalAtt.yaw_relative + att.yaw;
        if (gimbalAtt.yaw_absolute > FPI) gimbalAtt.yaw_absolute -= FPI;
        if (gimbalAtt.yaw_absolute < -FPI) gimbalAtt.yaw_absolute += FPI;
		}break;

    case MAVLINK_MSG_ID_MOUNT_STATUS: {
    	mavlink_mount_status_t payload;
        mavlink_msg_mount_status_decode(&_msg, &payload);
        gimbalAtt.roll = ((float)payload.pointing_b * 0.01f)*FDEGTORAD;
        gimbalAtt.pitch = ((float)payload.pointing_a * 0.01f)*FDEGTORAD;
        gimbalAtt.yaw_relative = ((float)payload.pointing_c * 0.01f)*FDEGTORAD;
        gimbalAtt.yaw_absolute = gimbalAtt.yaw_relative + att.yaw;
        if (gimbalAtt.yaw_absolute > FPI) gimbalAtt.yaw_absolute -= FPI;
        if (gimbalAtt.yaw_absolute < -FPI) gimbalAtt.yaw_absolute += FPI;
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
		autopilot.is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT;
		}break;

    case MAVLINK_MSG_ID_ATTITUDE: {
    	mavlink_attitude_t payload;
        mavlink_msg_attitude_decode(&_msg, &payload);
        att.roll = payload.roll;
        att.pitch = payload.pitch;
        att.yaw = payload.yaw;
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
        gps1.alt = payload.alt;
        gps1.vel = payload.vel;
        gps1.cog = payload.cog;
        gps_instancemask &= 0x01;
        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, GPS_ALT_FIRST_ID, 0, 10, (int32_t)(payload.alt), UNIT_METERS, 3);
        if (payload.vel != UINT16_MAX) {
        	setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, GPS_SPEED_FIRST_ID, 0, 11, (int32_t)(payload.vel), UNIT_METERS, 2);
        }
        // payload.cog //uint16 cdeg
        // { GPS_COURS_FIRST_ID, GPS_COURS_LAST_ID, 0, ZSTR_HDG, UNIT_DEGREE, 2 },
        // { GPS_LONG_LATI_FIRST_ID, GPS_LONG_LATI_LAST_ID, 0, ZSTR_GPS, UNIT_GPS, 0 },
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
        gps2.alt = payload.alt;
        gps2.vel = payload.vel;
        gps2.cog = payload.cog;
        gps_instancemask &= 0x02;
        }break;

    case MAVLINK_MSG_ID_VFR_HUD: {
    	mavlink_vfr_hud_t payload;
        mavlink_msg_vfr_hud_decode(&_msg, &payload);
    	vfr.airspd = payload.airspeed;
    	vfr.groundspd = payload.groundspeed;
    	vfr.alt = payload.alt;
    	vfr.climbrate = payload.climb;
    	vfr.heading = payload.heading;
    	vfr.thro = payload.throttle;
        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, ALT_FIRST_ID, 0, 13, (int32_t)(payload.alt * 100.0f), UNIT_METERS, 2);
        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, VARIO_FIRST_ID, 0, 14, (int32_t)(payload.climb * 100.0f), UNIT_METERS_PER_SECOND, 2);
        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, AIR_SPEED_FIRST_ID, 0, 15, (int32_t)(payload.airspeed * 100.0f), UNIT_METERS_PER_SECOND, 2);
        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, GPS_COURS_FIRST_ID, 0, 16, (int32_t)payload.heading * 10, UNIT_DEGREE, 1);
		}break;

    case MAVLINK_MSG_ID_GLOBAL_POSITION_INT: {
    	mavlink_global_position_int_t payload;
    	mavlink_msg_global_position_int_decode(&_msg, &payload);
        gposition.lat = payload.lat;
        gposition.lon = payload.lon;
        gposition.alt = payload.alt;
        gposition.relative_alt = payload.relative_alt;
        gposition.vx = payload.vx;
        gposition.vy = payload.vy;
        gposition.vz = payload.vz;
        gposition.hdg = payload.hdg;
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
    		bat1.charge_consumed = payload.current_consumed; // mAh, -1 if not known
    		bat1.energy_consumed = payload.energy_consumed; // 0.1 kJ, -1 if not known
    		bat1.temperature = payload.temperature; // centi-degrees C°, INT16_MAX if not known
    		bat1.voltage = voltage; // mV
    		bat1.current = payload.current_battery; // 10*mA, -1 if not known
    		//bat1.function = payload.battery_function;
    		//bat1.type = payload.type;
    		bat1.remainingpct = payload.battery_remaining; //(0%: 0, 100%: 100), -1 if not knwon
    		bat1.cellcount = cellcount;
        }
        if (payload.id == 1) {
    		bat2.charge_consumed = payload.current_consumed; // mAh, -1 if not known
    		bat2.energy_consumed = payload.energy_consumed; // 0.1 kJ, -1 if not known
    		bat2.temperature = payload.temperature; // centi-degrees C°, INT16_MAX if not known
    		bat2.voltage = voltage; // mV
    		bat2.current = payload.current_battery; // 10*mA, -1 if not known
    		bat2.remainingpct = payload.battery_remaining; //(0%: 0, 100%: 100), -1 if not knwon
    		bat2.cellcount = cellcount;
        }
        if (payload.id < 8) bat_instancemask &= (1 << payload.id);

        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, BATT_ID, 0, 17, voltage/100, UNIT_VOLTS, 1);
        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, VFAS_FIRST_ID, 0, 18, voltage/10, UNIT_VOLTS, 2);
    	int32_t current_battery = payload.current_battery / 10; //int16_t  cA
        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, CURR_FIRST_ID, 0, 19, current_battery, UNIT_AMPS, 1);
        //  { CELLS_FIRST_ID, CELLS_LAST_ID, 0, ZSTR_CELLS, UNIT_CELLS, 2 },
        // { RBOX_BATT1_FIRST_ID, RBOX_BATT1_LAST_ID, 0, ZSTR_BATT1_VOLTAGE, UNIT_VOLTS, 3 },
        // { RBOX_BATT1_FIRST_ID, RBOX_BATT1_LAST_ID, 1, ZSTR_BATT1_CURRENT, UNIT_AMPS, 2 },
        // { RBOX_CNSP_FIRST_ID, RBOX_CNSP_LAST_ID, 0, ZSTR_BATT1_CONSUMPTION, UNIT_MAH, 0 },
    	}break;

    case MAVLINK_MSG_ID_STATUSTEXT: {
        mavlink_statustext_t payload;
        mavlink_msg_statustext_decode(&_msg, &payload);
        payload.text[49] = '\0'; //terminate it properly, never mind losing the last char
        statustextFifo.push(payload);
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

	if (_msg.msgid != MAVLINK_MSG_ID_RADIO_STATUS) {
		_is_receiving = MAVLINK_TELEM_RECEIVING_TIMEOUT; //reset receiving timeout
	}

    if ((gimbal.compid == 0) && (_msg.msgid == MAVLINK_MSG_ID_HEARTBEAT)) {
		mavlink_heartbeat_t payload;
		mavlink_msg_heartbeat_decode(&_msg, &payload);
    	if ( (payload.autopilot == MAV_AUTOPILOT_INVALID) &&
    	     (	(payload.type == MAV_TYPE_GIMBAL) ||
    	    	((_msg.compid == MAV_COMP_ID_GIMBAL) ||
    	    			((_msg.compid >= MAV_COMP_ID_GIMBAL2) && (_msg.compid <= MAV_COMP_ID_GIMBAL6)))  )   ) {
            _resetGimbal();
    		gimbal.compid = _msg.compid;
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
        int32_t r = (payload.rssi == UINT8_MAX) ? 0 : payload.rssi;
        setTelemetryValue(PROTOCOL_TELEMETRY_FRSKY_SPORT, RSSI_ID, 0, 1, r, UNIT_DB, 0);
    	//#if defined(MULTIMODULE)
    	//{ TX_RSSI_ID, TX_RSSI_ID, 0, ZSTR_TX_RSSI   , UNIT_DB , 0 },
    	//{ TX_LQI_ID , TX_LQI_ID,  0, ZSTR_TX_QUALITY, UNIT_RAW, 0 },
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

void MavlinkTelem::wakeup()
{
#if defined(MAVLINK_TELEM)

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
            telemetryStreaming = 2*TELEMETRY_TIMEOUT10ms; // 2 seconds
	    }
	}

	// receiving timeouts
	if (_is_receiving) {
		_is_receiving--;
		if (!_is_receiving) _reset();
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
		if (mavlinkTelemPutBuf(_txbuf, _txcount)) _txcount = 0;
	}

#endif
}


// -- Miscellaneous --

// ArduPilot starts with sending heartbeat every 1 sec with FE, TimeSync every 5 sec with FE
// we then need to request the data stream
void MavlinkTelem::requestDataStreamFromAutopilot(void)
{
	if (autopilottype == MAV_AUTOPILOT_ARDUPILOTMEGA) {
		push_task(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTENDED_STATUS); // there is no clear_request() yet, so push task
		push_task(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_POSITION);
		push_task(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA1);
		push_task(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA2);
		push_task(TASK_AUTOPILOT, TASK_SENDREQUESTDATASTREAM_EXTRA3);

		push_task(TASK_AP, TASK_ARDUPILOT_REQUESTBANNER);
		return;
	}
/*
	// other autopilots
	SETTASK(TASK_SENDREQUESTDATASTREAM_RAW_SENSORS);
	SETTASK(TASK_SENDREQUESTDATASTREAM_EXTENDED_STATUS);
	SETTASK(TASK_SENDREQUESTDATASTREAM_POSITION);*/
}


void MavlinkTelem::_resetAutopilot(void)
{
    _task[TASK_AUTOPILOT] = 0;
    _task[TASK_AP] = 0;

    autopilot.compid = 0;
    autopilot.is_receiving = 0;
    autopilot.system_status = MAV_STATE_UNINIT;
	autopilot.custom_mode = 0;
    autopilot.is_armed = false;
    autopilot.is_standby = true;
    autopilot.is_critical = false;
    autopilot.prearm_ok = false;
    autopilot.requests_triggered = 0;
    autopilot.requests_tlast = 0;

	att.roll = 0.0f;
	att.pitch = 0.0f;
	att.yaw = 0.0f;

    gps1.fix = GPS_FIX_TYPE_NO_GPS;
    gps1.sat = UINT8_MAX;
    gps1.hdop = UINT16_MAX;
    gps1.vdop = UINT16_MAX;
    gps1.lat = 0;
    gps1.lon = 0;
    gps1.alt = 0;
    gps1.vel = UINT16_MAX;
    gps1.cog = UINT16_MAX;

    gps2.fix = GPS_FIX_TYPE_NO_GPS;
    gps2.sat = UINT8_MAX;
    gps2.hdop = UINT16_MAX;
    gps2.vdop = UINT16_MAX;
    gps2.lat = 0;
    gps2.lon = 0;
    gps2.alt = 0;
    gps2.vel = UINT16_MAX;
    gps2.cog = UINT16_MAX;

    gps_instancemask = 0;

 	gposition.relative_alt = 0;

	vfr.airspd = 0.0f;
	vfr.groundspd = 0.0f;
	vfr.alt = 0.0f;
	vfr.climbrate = 0.0f;
	vfr.heading = 0;
	vfr.thro = 0;

	bat1.charge_consumed = -1;
	bat1.energy_consumed = -1;
	bat1.temperature = INT16_MAX;
	bat1.voltage = 0;
	bat1.current = -1;
	bat1.remainingpct = -1;
	bat1.cellcount = -1;

	bat2.charge_consumed = -1;
	bat2.energy_consumed = -1;
	bat2.temperature = INT16_MAX;
	bat2.voltage = 0;
	bat2.current = -1;
	bat2.remainingpct = -1;
	bat2.cellcount = -1;

    bat_instancemask = 0;

    statustextFifo.clear();
}


void MavlinkTelem::_resetGimbal(void)
{
    _task[TASK_GIMBAL] = 0;

	gimbal.compid = 0;
	gimbal.is_receiving = 0;
	gimbal.system_status = MAV_STATE_UNINIT;
	gimbal.custom_mode = 0;
	gimbal.is_armed = false;
	gimbal.is_standby = true;
	gimbal.is_critical = false;
	gimbal.prearm_ok = false;
    gimbal.requests_triggered = 0;
    gimbal.requests_tlast = 0;

    gimbalAtt.roll = 0.0f;
    gimbalAtt.pitch = 0.0f;
    gimbalAtt.yaw_relative = 0.0f;
    gimbalAtt.yaw_absolute = 0.0f;
}


void MavlinkTelem::_resetCamera(void)
{
    _task[TASK_CAMERA] = 0;

    camera.compid = 0;
    camera.is_receiving = 0;
    camera.system_status = MAV_STATE_UNINIT;
	camera.custom_mode = 0;
    camera.is_armed = false;
    camera.is_standby = true;
    camera.is_critical = false;
    camera.prearm_ok = false;
    camera.requests_triggered = 0;
    camera.requests_tlast = 0;

	cameraInfo.vendor_name[0] = 0;
	cameraInfo.model_name[0] = 0;
	cameraInfo.flags = 0;
	cameraInfo.has_video = false;
	cameraInfo.has_photo = false;
	cameraInfo.has_modes = false;
	cameraInfo.total_capacity = NAN;
	cameraInfo.info_received = false;
	cameraInfo.settings_received = false;
	cameraInfo.status_received = false;

	cameraStatus.mode = 0;
	cameraStatus.video_on = false;
	cameraStatus.photo_on = false;
	cameraStatus.available_capacity = NAN;
	cameraStatus.recording_time_ms = UINT32_MAX;
	cameraStatus.battery_voltage = NAN;
	cameraStatus.battery_remainingpct = -1;

	cameraStatus.initialized = false;
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
    autopilottype = MAV_AUTOPILOT_GENERIC;
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
