#!/usr/bin/env python
'''
mavgenerate_dialect.py
calls mavlink generator for C header files
(c) olliw, olliw42
'''

#options to set

#mavlinkdialect = "storm32"
mavlinkdialect = "opentx"

mavlinkoutputdirectory = 'out'

mavlinkpathtorepository = r'fastmavlink'


'''
Imports
'''
import os
import shutil
import re
import sys

#we may have not installed it or have different pymavlink, so set things straight
sys.path.insert(0,mavlinkpathtorepository)

from fastmavlink.generator import fmavgen


'''
Generates the header files and place them in the output directory.
'''

outdir = mavlinkoutputdirectory
        
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
        opts = fmavgen.Opts(outdir)
        args = [xmfile]
        try:
            fmavgen.fmavgen(opts,args)
            print('Successfully Generated Headers', 'Headers generated successfully.')

        except Exception as ex:
            exStr = str(ex)
            print('Error Generating Headers','{0!s}'.format(exStr))
            exit()

