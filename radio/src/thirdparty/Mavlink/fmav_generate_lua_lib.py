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

'''
Attention: names must not be longer than 32 chars
'''


def excludeMessage(msg):
    #return True
    return False
    if msg.name in ['AHRS','ATTITUDE','VIBRATION']: return False
    #if msg.name in ['AHRS']: return False
    return True


def shortenName(name, width):
    nameshort = name[:]
    if len(nameshort) > width:
        print('WARNING: msg id '+name+' too long')
        nn = str.split(name, '_')
        nameshort = nn[0]+'_'
        for i in range(1,len(nn)-1):
            nameshort += nn[i][:3] + '_'
        nameshort += nn[-1]
        if len(nameshort) > width:
            print(' ! ! !   msg id too long even after shortening')
            nameshort = nameshort[:width]
    return nameshort 

def shortenNameEnum(name, width):
    nameshort = name[:]
    if nameshort[:4] == 'MAV_': nameshort = nameshort[4:]
    if len(nameshort) > width:
        print('WARNING: msg id '+name+' too long')
        nn = str.split(name, '_')
        nameshort = nn[0]+'_'
        for i in range(1,len(nn)-1):
            nameshort += nn[i][:3]
        nameshort += '_' + nn[-1]
        if len(nameshort) > width:
            print(' ! ! !   enum field too long even after shortening')
            nameshort = nameshort[:width]
    return nameshort 


def generateLibMessageStr(msg, m):
    m.append('  case FASTMAVLINK_MSG_ID_'+msg.name+': {')
    count = 0
    for field in msg.fields:
        if field.array_length == 0:
            if msg.is_target_system_field(field): continue
            if msg.is_target_component_field(field): continue
            count += 1
        else:    
            count += 1
    if count > 0:
        m.append('    fmav_'+msg.name.lower()+'_t* payload = (fmav_'+msg.name.lower()+'_t*)(mavmsg->payload_ptr);')
    #for field in msg.ordered_fields:
    for field in msg.fields:
        if field.array_length == 0:
            #we strip of target fields as their values are already in the msg structure
            if msg.is_target_system_field(field): continue
            if msg.is_target_component_field(field): continue
            #m.append('    push_value(L, "'+field.name+'", payload.'+field.name+');')
            #m.append('    lua_pushtableinteger(L, "'+field.name+'", payload->'+field.name+');')
            nameshort = shortenName(field.name, 32)
            m.append('    lua_pushtablenumber(L, "'+nameshort+'", payload->'+field.name+');')
        else:
            nameshort = shortenName(field.name, 32)
            m.append('    lua_pushstring(L, "'+nameshort+'"); // array '+field.name+'['+str(field.array_length)+']' )
            m.append('    lua_newtable(L);')
            m.append('    for (int i = 0; i < '+str(field.array_length)+'; i++) { ')
            #m.append('      push_value(L, i, payload.'+field.name+'[i]);')
            m.append('      lua_pushtableinumber(L, i, payload->'+field.name+'[i]);')
            m.append('    }')
            m.append('    lua_rawset(L, -3);')
    m.append('    break;')
    m.append('    }')


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


#define lua_pushtableinteger_raw(L, k, v)  (lua_pushstring(L, (k)), lua_pushinteger(L, (v)), lua_rawset(L, -3))
#define lua_pushtablenumber_raw(L, k, v)   (lua_pushstring(L, (k)), lua_pushnumber(L, (v)), lua_rawset(L, -3))
#define lua_pushtablestring_raw(L, k, v)   (lua_pushstring(L, (k)), lua_pushstring(L, (v)), lua_rawset(L, -3))
#define lua_pushtableinumber_raw(L, i, v)  (lua_pushnumber(L, (i)), lua_pushnumber(L, (v)), lua_rawset(L, -3))
#define lua_pushtableinumber(L, i, v)      (lua_pushnumber(L, (i)), lua_pushnumber(L, (v)), lua_settable(L, -3))

static void luaMavlinkPushMavMsg(lua_State *L, MavlinkTelem::MavMsg* mavmsg)
{
  lua_newtable(L);
  lua_pushtableinteger(L, "sysid", mavmsg->sysid);
  lua_pushtableinteger(L, "compid", mavmsg->compid);
  lua_pushtableinteger(L, "msgid", mavmsg->msgid);
  lua_pushtableinteger(L, "target_sysid", mavmsg->target_sysid);
  lua_pushtableinteger(L, "target_compid", mavmsg->target_compid);
  lua_pushtableboolean(L, "updated", mavmsg->updated);
  mavmsg->updated = false;

  switch (mavmsg->msgid) {''')
    for msgid in sorted(dialectxml.messages_all_by_id.keys()):
        msg = dialectxml.messages_all_by_id[msgid]
        if excludeMessage(msg): continue
        generateLibMessageStr(msg, m)
    m.append('''  }
}
''')

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
        #s.append('  { "MSG_ID_'+name+'", FASTMAVLINK_MSG_ID_'+name+' }, \\')
        nameshort = shortenName(name, 30)
        s.append('  { "M_'+nameshort+'", FASTMAVLINK_MSG_ID_'+name+' }, \\')
    for enumname in enums_all_by_name:
        #print(enumname)
        s.append( '  \\')
        for entry in enums_all_by_name[enumname].entry:
            #print('    ',entry.name, entry.value)
            #s.append('  { "'+entry.name+'", '+str(entry.value)+' }, \\')
            nameshort = shortenNameEnum(entry.name, 32)
            s.append('  { "'+nameshort+'", '+str(entry.value)+' }, \\')
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
