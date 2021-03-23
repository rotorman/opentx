#!/usr/bin/env python
'''
fmav_generate_lua-lib.py
calls fastMavlink generator modules
(c) OlliW, OlliW42, www.olliw.eu
'''
import os
from fastmavlink.generator.modules import fmav_parse as mavparse
from fastmavlink.generator.modules import mavtemplate
from fastmavlink.generator.modules import fmav_flags as mavflags


def generateLibMessageStr(msg, m):
    m.append('void luaMavlinkGet_'+msg.name+'(lua_State* L, const fmav_message_t* msg)')
    m.append('{')
    m.append('  fmav_'+msg.name.lower()+'_t payload;')
    m.append('  fmav_msg_'+msg.name.lower()+'_decode(&payload, msg);')
    m.append('')
    m.append('  lua_newtable(L);')
    m.append('  luaMavlinkPushHeader(L, msg);')
    #m.append('')

    #for field in msg.ordered_fields:
    for field in msg.fields:
        if field.array_length == 0:
            #we strip of target fields as their values are already in the msg structure
            if msg.is_target_system_field(field): continue
            if msg.is_target_component_field(field): continue
            m.append('  push_value(L, "'+field.name+'", payload.'+field.name+');')
        else:
            m.append('  lua_pushstring(L, "'+field.name+'"); // array '+field.name+'['+str(field.array_length)+']' )
            m.append('  lua_newtable(L);')
            m.append('  for (int i = 0; i < '+str(field.array_length)+'; i++) { ')
            m.append('    push_value(L, i, payload.'+field.name+'[i]);')
            m.append('  }')
            m.append('  lua_rawset(L, -3); // end array')

    #m.append('')
    m.append('  push_value(L, "updated", true);')
    m.append('  lua_setglobal(L, "mavlink_msg_'+msg.name.lower()+'");')
    m.append('}')
    m.append('')


def generateLuaLibHeaders(dialectname):
    print("Run XML %s" % os.path.basename(dialectname))
    xml_list = mavparse.generateXmlList(dialectname)
    messagesoutname = os.path.splitext(os.path.basename(dialectname))[0] + '_lua_lib_messages.h'
    constantsoutname = os.path.splitext(os.path.basename(dialectname))[0] + '_lua_lib_constants.h'

    for xml in xml_list:
        print(xml.basename)
    print("Found %u messages and %u enums in %u XML files" %
        (mavparse.totalNumberOfMessages(xml_list), mavparse.totalNumberOfEnums(xml_list), len(xml_list)))

    # find dialectxml, and generate complete enum dictionary
    dialectxml = None
    enums_all_by_name = {}
    for xml in xml_list:
        if xml.basename == 'opentx':
            dialectxml = xml
        for enum in xml.enums_merged:
            if not enum.name in enums_all_by_name.keys():
                enums_all_by_name[enum.name] = enum

    print("Messages")
    print('->',dialectxml.basename)
    m = []
    m.append('''//------------------------------------------------------------
// mavlink messages
//------------------------------------------------------------
// all message from opentx.xml
// auto generated

// idea is by hsteinhaus
// https://github.com/opentx/opentx/pull/5600


//------------------------------------------------------------
//-- Helper
//------------------------------------------------------------

void push_value(lua_State* L, const char* field, const char* text)
{
  lua_pushstring(L, field);
  lua_pushstring(L, text);
  lua_rawset(L, -3);
}

void push_value(lua_State* L, const char* field, const double value)
{
  lua_pushstring(L, field);
  lua_pushnumber(L, value);
  lua_rawset(L, -3);
}

void push_value(lua_State* L, const int i, const char* text)
{
  lua_pushnumber(L, i);
  lua_pushstring(L, text);
  lua_rawset(L, -3);
}

void push_value(lua_State* L, const int i, const double value)
{
  lua_pushnumber(L, i);
  lua_pushnumber(L, value);
  lua_rawset(L, -3);
}

void luaMavlinkPushHeader(lua_State* L, const fmav_message_t* msg)
{
  push_value(L, "magic", msg->magic);
  push_value(L, "len", msg->len);
  push_value(L, "incompat_flags", msg->incompat_flags);
  push_value(L, "compat_flags", msg->compat_flags);
  push_value(L, "seq", msg->seq);
  push_value(L, "sysid", msg->sysid);
  push_value(L, "compid", msg->compid);
  push_value(L, "target_sysid", msg->target_sysid);
  push_value(L, "target_compid", msg->target_compid);
}


//------------------------------------------------------------
//-- Messages
//------------------------------------------------------------
''')
    for msgid in sorted(dialectxml.messages_all_by_id.keys()):
        generateLibMessageStr(dialectxml.messages_all_by_id[msgid], m)
    m.append('''
//------------------------------------------------------------
//-- Message Handler
//------------------------------------------------------------

void luaMavlinkHandleMessage(const fmav_message_t* msg)
{
  lua_State* L = lsWidgets;
''')
    m.append('  switch (msg->msgid) {')
    for msgid in sorted(dialectxml.messages_all_by_id.keys()):
        name = dialectxml.messages_all_by_id[msgid].name
        m.append('    case FASTMAVLINK_MSG_ID_'+name+':')
        m.append('      luaMavlinkGet_'+name+'(L, msg);')
        m.append('      return;')
    m.append('  }')
    m.append('}')

    # constants
    print("Constants")
    print('->',dialectxml.basename)
    s = []
    s.append('''//------------------------------------------------------------
// mavlink constants
//------------------------------------------------------------
// all message and enum constants from opentx.xml
// auto generated

#define MAVLINK_LIB_CONSTANTS \\''')
    for msgid in sorted(dialectxml.messages_all_by_id.keys()):
        #print(msgid, dialectxml.messages_all_by_id[msgid].name)
        name = dialectxml.messages_all_by_id[msgid].name
        s.append('  { "MSG_ID_'+name+'", FASTMAVLINK_MSG_ID_'+name+' }, \\')
    for enumname in enums_all_by_name:
        #print(enumname)
        s.append( '  \\')
        for entry in enums_all_by_name[enumname].entry:
            #print('    ',entry.name, entry.value)
            s.append('  { "'+entry.name+'", '+str(entry.value)+' }, \\')
    #print(s)

    F = open(constantsoutname, mode='w')
    for ss in s:
        F.write(ss)
        F.write('\n')
    F.write('\n')
    F.close()

    F = open(messagesoutname, mode='w')
    for mm in m:
        F.write(mm)
        F.write('\n')
    F.write('\n')
    F.close()


if __name__ == "__main__":
    dialectname = os.path.join('opentx.xml')
    generateLuaLibHeaders(dialectname)
