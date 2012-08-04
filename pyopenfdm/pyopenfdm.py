#    pyopenfdm.py: wrap openfdm python for better error handling
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
from os.path import join, getsize
import tempfile
import sys
import argparse
import logging
import re

# find root path or project
root_path = os.path.abspath(os.path.join(
    inspect.getfile(inspect.currentframe()),
    os.path.pardir,os.path.pardir))

# set enviornment to include project root
os.environ['OPENMODELICALIBRARY'] = os.getenv('OPENMODELICALIBRARY') + ':' + root_path

# create build directory and move to it
build_path = os.path.join(tempfile.gettempdir(),'ompython')
if not os.path.isdir(build_path):
    os.makedirs(build_path)
os.chdir(build_path)

import OMPython


re_error = re.compile('.*error|fail.*',re.I|re.M)
re_package = re.compile('within[\s]+([\w.]+)')
re_model = re.compile('model[\s]+([\w.]+)')

def om_shell(command):
    result = OMPython.execute(command)
    error = OMPython.execute('''getErrorString()''').strip()[1:-1]
    
    if type(result) is bool:
        if not (result):
            raise Exception("command: %s returned false" % command)
    elif type(result) is dict:
        messages = OMPython.get(result,'SimulationResults.messages')
        print messages
        if re_error.match(messages):
            raise Exception(messages)
    elif type(result) is str:
        if re_error.match(result):
            raise Exception(result)

    if re_error.match(error):
        raise Exception(error)

    return result

def run_script(script):
    om_shell('''clear()''')
    result = om_shell('''runScript("%s")''' % (script))
    print result
    return result

def run_model(model):
    om_shell('clear()')
    om_shell('loadModel(Modelica)')
    om_shell('loadModel(MultiBodyOmc)')
    om_shell('loadModel(OpenFDM)')
    om_shell('loadModel(test)')
    packageName = None
    modelName = None
    with open(model) as f:
        lines = f.readlines();    
    for line in lines:
        result = re_package.match(line)
        if result:
            packageName = result.group(1)
        result = re_model.match(line)
        if result:
            modelName = result.group(1)    
    if not packageName:
        raise Exception("package name not found in " + model)
    if not modelName:
        raise Exception("model name not found in " + model)
    return om_shell('''simulate(%s.%s)''' % (packageName,modelName))

def find_models(root_path):
    for root, dirs, files in os.walk(root_path):
        for fileName in files:
            if fileName != "package.mo" and os.path.splitext(fileName)[1] == ".mo":
                yield os.path.join(root,fileName)

def find_scripts(root_path):
    for root, dirs, files in os.walk(root_path):
        for fileName in files:
            if os.path.splitext(fileName)[1] == ".mos":
                yield os.path.join(root,fileName)
