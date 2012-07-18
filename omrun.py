#!/usr/bin/env python
#
#    Filename: omrun.py: python script to improve OMPython script interface
#    Copyright (C) 2012  James Goppert <james.goppert@gmail.com>
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http:#www.gnu.org/licenses/>.

import os
import inspect
import tempfile
import argparse
import logging

# parse command line
parser = argparse.ArgumentParser(description='Runs modelica script using python.')
parser.add_argument('script', metavar='script', help='script to run')
parser.add_argument('-e', '--echo', default=False, action='store_true', help='enable echo')
parser.add_argument('-q', '--quiet', default=False, action='store_true', help='quiet mode')
args = parser.parse_args()

# handle logging
if args.quiet:
    log_level = logging.WARNING
else:
    log_level = logging.INFO
logging.basicConfig(format='%(message)s',level=log_level)

# find root path or project
root_path = os.path.abspath(os.path.join(inspect.getfile(inspect.currentframe()),os.path.pardir))
script_path = os.path.abspath(args.script)

# check that script exists
if not os.path.isfile(script_path):
    raise IOError("script %s not found" % script_path)

# set enviornment to include project root
os.environ['OPENMODELICALIBRARY'] = os.getenv('OPENMODELICALIBRARY') + ':' + root_path

# create build directory and move to it
build_path = os.path.join(tempfile.gettempdir(),'ompython')
if not os.path.isdir(build_path):
    os.makedirs(build_path)
os.chdir(build_path)

# import OMPython now that the environment is setup correctly and we have a script
import OMPython

# add project root to modelica environment path
logging.info('OPENMODELICALIBRARY: %s' % OMPython.execute('''getEnvironmentVar("OPENMODELICALIBRARY")''').strip())
logging.info('working directory: %s' % OMPython.execute('''cd()''').strip())
logging.info('script path: "%s"' % script_path)

# run script
#print 'running script: \n', OMPython.execute('''runScript("%s")''' % script_path).strip()[1:-1]

# get next line of script
def get_next_line(stringio):
    line = []
    while True:
        byte = stringio.read(1)
        if not byte:
            break
        line.append(byte)
        if byte == ';':
            break
    return ''.join(line).strip()

# run script line by line checking for error
with open(script_path,'r') as script_file:
    while True:
        line = get_next_line(script_file)
        if args.echo:
            print line
        if not line:
            logging.info('%s completed successfully without errors' % script_path)
            break
        result = OMPython.execute(line)
        error = OMPython.execute('''getErrorString()''').strip()[1:-1]
        if error:
            raise Exception(error)
