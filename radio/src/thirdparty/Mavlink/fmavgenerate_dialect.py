#!/usr/bin/env python
'''
fmavgenerate_dialect.py
calls fastMavlink generator for C header files
(c) olliw, olliw42
'''
import os
import shutil
import re
import sys


#options to set

#mavlinkpathtorepository = os.path.join("C:/",'Users','Olli','Documents','GitHub','fastmavlink')
mavlinkpathtorepository = os.path.join('..','..','..','..','..','fastmavlink')

#mavlinkdialect = "\..\mavlink\external\dialects\storm32"
mavlinkdialect = "opentx"

mavlinkoutputdirectory = 'out'
#mavlinkoutputdirectory = 'v2.0'


'''
Imports
'''
#import pkgutil
#search_path = [mavlinkpathtorepository] # set to None to see all modules importable from sys.path
#all_modules = [x[1] for x in pkgutil.iter_modules(path=search_path)]
#print(all_modules)

#we may have not installed it or have different fastMavlink, so set things straight
sys.path.insert(0,mavlinkpathtorepository)

from generator import fmavgen
from generator.modules import fmavflags

'''
Generates the header files and place them in the output directory.
'''

outdir = mavlinkoutputdirectory

#xmfile = os.path.abspath(os.path.join(mavlinkpathtorepository, mavlinkdialect+'.xml'))
xmfile = mavlinkdialect+'.xml'

#recreate out directory
print('----------')
print('kill out dir')
try:
    shutil.rmtree(outdir)
except:
    pass    
os.mkdir(outdir)
print('----------')

opts = fmavgen.Opts(outdir, parse_flags=fmavflags.PARSE_FLAGS_WARNING_ENUM_VALUE_MISSING)
args = [xmfile]
try:
    fmavgen.fmavgen(opts,args)
    print('Successfully Generated Headers', 'Headers generated successfully.')

except Exception as ex:
    exStr = str(ex)
    print('Error Generating Headers','{0!s}'.format(exStr))
    exit()

