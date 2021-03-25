/*
* (c) www.olliw.eu, OlliW, OlliW42
*/

#include "opentx.h"

// -- Receive stuff --

void MavlinkTelem::mavMsgListInit(void)
{
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) _mavMsgList[i] = NULL;
  _mavmsg_enabled = false;
}

void MavlinkTelem::mavMsgListEnable(bool flag)
{
  _mavmsg_enabled = flag;
}

uint8_t MavlinkTelem::mavMsgListCount(void)
{
  if (!_mavmsg_enabled) return 0;

  uint8_t cnt = 0;
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) if(_mavMsgList[i]) cnt++;
  return cnt;
}

// we probably need to differentiate not only by msgid, but also by sysis-compid
// if two components send the same message at (too) high rate considering only msgid leads to message loss

uint8_t MavlinkTelem::_mavMsgListFindOrAdd(uint32_t msgid)
{
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) {
    if (!_mavMsgList[i]) continue;
    if (_mavMsgList[i]->msgid == msgid) { return i; }
  }
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) {
    if (_mavMsgList[i]) continue;
    // free spot, so add it
    _mavMsgList[i] = (MavMsg*)malloc(sizeof(MavMsg));
    if (!_mavMsgList[i]) return UINT8_MAX; // grrrr
    _mavMsgList[i]->msgid = msgid;
    _mavMsgList[i]->payload_ptr = NULL;
    _mavMsgList[i]->updated = false;
    return i;
  }
  return UINT8_MAX;
}

void MavlinkTelem::handleMavapiMessage(fmav_message_t* msg)
{
  if (!_mavmsg_enabled) return;

  uint8_t i = _mavMsgListFindOrAdd(msg->msgid);
  if (i == UINT8_MAX) return;

  if (!_mavMsgList[i]->payload_ptr) _mavMsgList[i]->payload_ptr = malloc(msg->payload_max_len);
  if (!_mavMsgList[i]->payload_ptr) return; // grrrr

  _mavMsgList[i]->sysid = msg->sysid;
  _mavMsgList[i]->compid = msg->compid;
  _mavMsgList[i]->target_sysid = msg->target_sysid;
  _mavMsgList[i]->target_compid = msg->target_compid;
  memcpy(_mavMsgList[i]->payload_ptr, msg->payload, msg->payload_max_len);
  _mavMsgList[i]->updated = true;
  _mavMsgList[i]->timestamp = time10us();
}

MavlinkTelem::MavMsg* MavlinkTelem::mavMsgListGet(uint32_t msgid)
{
  if (!_mavmsg_enabled) return NULL;

  uint8_t i_found = UINT8_MAX;
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) {
    if (!_mavMsgList[i]) continue;
    if (!_mavMsgList[i]->payload_ptr) continue; // it must have been received completely
    if (_mavMsgList[i]->msgid == msgid) { i_found = i; }
  }
  if (i_found == UINT8_MAX) return NULL;
  return _mavMsgList[i_found];
}

MavlinkTelem::MavMsg* MavlinkTelem::mavMsgListGetLast(void)
{
  if (!_mavmsg_enabled) return NULL;

  uint32_t t_max = 0;
  uint8_t i_found = UINT8_MAX;
  for (uint8_t i = 0; i < MAVMSGLIST_MAX; i++) {
    if (!_mavMsgList[i]) continue;
    if (!_mavMsgList[i]->payload_ptr) continue; // it must have been received completely
    if (_mavMsgList[i]->timestamp > t_max) { t_max = _mavMsgList[i]->timestamp; i_found = i; }
  }
  if (i_found == UINT8_MAX) return NULL;
  return _mavMsgList[i_found];
}

// -- Send stuff --

fmav_message_t* MavlinkTelem::mavMsgOutPtr(void)
{
  if (!_mavapi_msg_out_free) return NULL;
  return &_t_mavapi_msg_out;
}

void MavlinkTelem::mavMsgOutSet(void)
{
  _mavapi_msg_out_free = false;
  SETTASK(TASK_ME, TASK_SENDMSG_MAVLINK_API);
}

void MavlinkTelem::generateMavapiMessage(void)
{
  memcpy(&_msg_out, &_t_mavapi_msg_out, sizeof(fmav_message_t));
  fmav_finalize_msg(&_msg_out, &_status_out);
  _msg_out_available = true;
  _mavapi_msg_out_free = true;
}
