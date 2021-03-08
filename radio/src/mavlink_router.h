/*
 * (c) www.olliw.eu, OlliW, OlliW42
 */


#include "thirdparty/Mavlink/out/fastmavlink_router.h"

// a simple wrapper class to have it nice

class MavlinkRouter
{
  public:
    MavlinkRouter() { fmav_router_reset(); } // constructor

    void reset(void) { fmav_router_reset(); }

    void handleMessage(uint8_t link_of_msg, fmav_result_t* result)
    {
      fmav_router_handle_message(link_of_msg, result);
    }

    void handleMessage(uint8_t link_of_msg, fmav_message_t* msg)
    {
      fmav_router_handle_message_by_msg(link_of_msg, msg);
    }

#if 0

    void handleMessage(uint8_t link_of_msg, mavlink_message_t* msg)
    {
      // determine targets
      uint8_t target_sysid = 0;
      uint8_t target_compid = 0;
      const mavlink_msg_entry_t* msg_entry = mavlink_get_msg_entry(msg->msgid);
      if (msg_entry) {
        if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_SYSTEM)
          target_sysid = (uint8_t)_MAV_PAYLOAD(msg)[msg_entry->target_system_ofs];
        if (msg_entry->flags & MAV_MSG_ENTRY_FLAG_HAVE_TARGET_COMPONENT)
          target_compid = (uint8_t)_MAV_PAYLOAD(msg)[msg_entry->target_component_ofs];
      }

      fmav_result_t result;
      result.msgid = msg->msgid;
      result.sysid = msg->sysid;
      result.compid = msg->compid;
      result.target_sysid = target_sysid;
      result.target_compid = target_compid;

      fmav_router_handle_message(link_of_msg, &result);
    }
#endif

    bool sendToLink(uint8_t link)
    {
      return (fmav_router_send_to_link(link)) ? true : false;
    }

    void addOurself(uint8_t sysid, uint8_t compid)
    {
      fmav_router_add_ourself(sysid, compid); //we always add ourself as link 0
    }

    void clearoutLink(uint8_t link)
    {
      fmav_router_clearout_link(link);
    }
};


