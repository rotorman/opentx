#!/usr/bin/env python
'''
fmavgenerate_dialect.py
calls fastMavlink generator for C header files
(c) olliw, olliw42
'''

#options to set

#mavlinkpathtorepository = r'C:\Users\Olli\Documents\GitHub\fastmavlink'
mavlinkpathtorepository = r'fastmavlink'

#mavlinkdialect = "\..\mavlink\external\dialects\storm32"
mavlinkdialect = "opentx"

mavlinkoutputdirectory = 'out'
#mavlinkoutputdirectory = 'v2.0'


'''
Imports
'''
import os
import shutil
import re
import sys

#import pkgutil
#search_path = [mavlinkpathtorepository] # set to None to see all modules importable from sys.path
#all_modules = [x[1] for x in pkgutil.iter_modules(path=search_path)]
#print(all_modules)

#we may have not installed it or have different fastMavlink, so set things straight
sys.path.insert(0,mavlinkpathtorepository)

from fastmavlink.generator import fmavgen
from fastmavlink.generator.modules import fmavflags

'''
Generates the header files and place them in the output directory.
'''

outdir = mavlinkoutputdirectory
        
#xmfile = os.path.abspath(mavlinkpathtorepository + mavlinkdialect+'.xml')
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

if True:
        opts = fmavgen.Opts(outdir, warning_flags=fmavflags.WARNING_FLAGS_ENUM_VALUE_MISSING)
        args = [xmfile]
        try:
            fmavgen.fmavgen(opts,args)
            print('Successfully Generated Headers', 'Headers generated successfully.')

        except Exception as ex:
            exStr = str(ex)
            print('Error Generating Headers','{0!s}'.format(exStr))
            exit()

