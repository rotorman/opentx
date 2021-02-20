/*
 * (c) www.olliw.eu, OlliW, OlliW42
 */


#define MAVLINK_ROUTER_LINKS_MAX  4
#define MAVLINK_ROUTER_COMPONENTS_MAX  12


class MavlinkRouter
{
  public:
    MavlinkRouter() { _reset(); } // constructor

    void reset(void) { _reset(); }

#if !defined(MAVLINK_COMMAND_24BIT)
    void handleMessage(uint8_t link_of_msg, fmav_result_t* result)
    {
      if (link_of_msg >= MAVLINK_ROUTER_LINKS_MAX) {
        for(uint8_t link = 0; link < MAVLINK_ROUTER_LINKS_MAX; link++) _send_to_link[link] = false;
        return;
      }

      // keep list of available components by spying heartbeats
      // heartbeats are always send to all links
      if (result->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        for(uint8_t link = 0; link < MAVLINK_ROUTER_LINKS_MAX; link++) _send_to_link[link] = true;
        _send_to_link[link_of_msg] = false; //origin of msg, don't reflect it back
        findOrAddComponent(link_of_msg, result->sysid, result->compid);
        return;
      }

      // determine to which links it has to be send
      for (uint8_t link = 0; link < MAVLINK_ROUTER_LINKS_MAX; link++) {
        _send_to_link[link] = false;

        if (link == link_of_msg) continue; //origin of msg, don't reflect it back

        if (accept(link, result->target_sysid, result->target_compid)) {
          _send_to_link[link] = true;
          continue;
        }
      }
    }
#endif

#if defined(FASTMAVLINK_PYMAVLINK_ENABLED) or defined(MAVLINK_COMMAND_24BIT)
    void handleMessage(uint8_t link_of_msg, mavlink_message_t* msg)
    {
      if (link_of_msg >= MAVLINK_ROUTER_LINKS_MAX) {
        for(uint8_t link = 0; link < MAVLINK_ROUTER_LINKS_MAX; link++) _send_to_link[link] = false;
        return;
      }

      // keep list of available components by spying heartbeats
      // heartbeats are always send to all links
      if (msg->msgid == MAVLINK_MSG_ID_HEARTBEAT) {
        for(uint8_t link = 0; link < MAVLINK_ROUTER_LINKS_MAX; link++) _send_to_link[link] = true;
        _send_to_link[link_of_msg] = false; //origin of msg, don't reflect it back
        findOrAddComponent(link_of_msg, msg->sysid, msg->compid);
        return;
      }

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

      // determine to which links it has to be send
      for (uint8_t link = 0; link < MAVLINK_ROUTER_LINKS_MAX; link++) {
        _send_to_link[link] = false;

        if (link == link_of_msg) continue; //origin of msg, don't reflect it back

        if (accept(link, target_sysid, target_compid)) {
          _send_to_link[link] = true;
          continue;
        }
      }
    }
#endif

    bool sendToLink(uint8_t link)
    {
      if (link >= MAVLINK_ROUTER_LINKS_MAX) return false;
      return _send_to_link[link];
    }

    void addOurself(uint8_t sysid, uint8_t compid)
    {
      addComponent(0, sysid, compid); //we always add ourself as link 0
    }

    void clearoutLink(uint8_t link)
    {
      if (link >= MAVLINK_ROUTER_LINKS_MAX) return;

      for (uint8_t i = 0; i < MAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (!componentList[i].valid) continue; //empty entry
        if (componentList[i].link == link) { //clear out
          componentList[i].valid = false;
          componentList[i].link = 0;
          componentList[i].sysid = 0;
          componentList[i].compid = 0;
        }
      }
    }

  private:

    struct ComponentItem {
      bool valid;
      uint8_t sysid;
      uint8_t compid;
      uint8_t link; // 0 is ourself, 1 = COMM0, 2 = COMM1, ...
    };

    struct ComponentItem componentList[MAVLINK_ROUTER_COMPONENTS_MAX];

    bool _send_to_link[MAVLINK_ROUTER_LINKS_MAX] = {false};

    void _reset(void)
    {
      for (uint8_t i = 0; i < MAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        componentList[i].valid = false;
        componentList[i].link = 0;
        componentList[i].sysid = 0;
        componentList[i].compid = 0;
      }
    }

    bool accept(uint8_t target_link, uint8_t target_sysid, uint8_t target_compid)
    {
      // go through all components on the link and see if the one we are targeting at is there
      for (uint8_t i = 0; i < MAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (!componentList[i].valid) continue;

        if (componentList[i].link != target_link) continue; //not our link

        if (target_sysid == 0) { //target link has seen at least one component, and target_sysid is broadcast, so ok
          return true;
        }

        if (componentList[i].sysid != target_sysid) continue; //not our system

        if (target_compid == 0) { //target_sysid is on the link, and target_compid is broadcast
          return true;
        }

        if (componentList[i].compid == target_compid) { //target_sysid and target_compid is on the link
          return true;
        }
      }

      return false;
    }

    bool findComponent(uint8_t sysid, uint8_t compid)
    {
      for (uint8_t i = 0; i < MAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (componentList[i].valid && componentList[i].sysid == sysid && componentList[i].compid == compid) {
          return true;
        }
      }
      return false;
    }

    bool addComponent(uint8_t link, uint8_t sysid, uint8_t compid)
    {
      if (link >= MAVLINK_ROUTER_LINKS_MAX) return false;

      for (uint8_t i = 0; i < MAVLINK_ROUTER_COMPONENTS_MAX; i++) {
        if (componentList[i].valid) continue; //already occupied
        componentList[i].valid = true;
        componentList[i].link = link;
        componentList[i].sysid = sysid;
        componentList[i].compid = compid;
        return true;
      }
      return false;
    }

    bool findOrAddComponent(uint8_t link, uint8_t sysid, uint8_t compid)
    {
      if (findComponent(sysid, compid)) return true;
      // not found, so try to add
      if (addComponent(link, sysid, compid)) return true;
      // could not be added
      return false;
    }

};

