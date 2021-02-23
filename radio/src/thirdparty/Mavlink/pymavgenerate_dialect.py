#!/usr/bin/env python
'''
mavgenerate_dialect.py
calls mavlink generator for C header files
(c) olliw, olliw42
'''
import os
import shutil
import re
import sys


#options to set

mavlinkpathtorepository = os.path.join('mavlink')

#mavlinkdialect = "storm32"
mavlinkdialect = "opentx"

mavlinkoutputdirectory = 'out'


'''
Imports
'''
#import pkgutil
#search_path = [mavlinkpathtorepository] # set to None to see all modules importable from sys.path
#all_modules = [x[1] for x in pkgutil.iter_modules(path=search_path)]
#print(all_modules)

#we may have not installed it or have different pymavlink, so set things straight
sys.path.insert(0,mavlinkpathtorepository)

from pymavlink.generator import mavgen
from pymavlink.generator import mavparse

'''
Generates the header files and place them in the output directory.
'''

outdir = mavlinkoutputdirectory

xmfile = mavlinkdialect+'.xml'

wire_protocol = mavparse.PROTOCOL_2_0
language = 'C'
validate = mavgen.DEFAULT_VALIDATE
error_limit = 5
strict_units = mavgen.DEFAULT_STRICT_UNITS

#recreate out directory
print('----------')
print('kill out dir')
try:
    shutil.rmtree(outdir)
except:
    pass
os.mkdir(outdir)
print('----------')

opts = mavgen.Opts(outdir, wire_protocol=wire_protocol, language=language, validate=validate, error_limit=error_limit, strict_units=strict_units)
args = [xmfile]
try:
    mavgen.mavgen(opts,args)
    print('Successfully Generated Headers', 'Headers generated successfully.')

except Exception as ex:
    exStr = str(ex)
    print('Error Generating Headers','{0!s}'.format(exStr))
    exit()

