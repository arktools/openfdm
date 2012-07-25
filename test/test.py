from pyopenfdm import OMInterface
import os
import inspect
from os.path import join, getsize
import tempfile
import sys
import argparse
import logging
import re
from nose.tools import with_setup;

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

def find_examples():
    for root, dirs, files in os.walk(os.path.join(root_path,'OpenFDM')):
        if os.path.basename(root) == "Examples":
            for fileName in files:
                if fileName != "package.mo" and os.path.splitext(fileName)[1] == ".mo":
                    yield os.path.join(root,fileName)

def setup_examples():
    execute("loadModel(Modelica)")
    execute("loadModel(OpenFDM)")

def teardown_examples():
    pass

@with_setup(setup_examples, teardown_examples)
def test_examples():
    for example in find_examples():
        yield run_example, example

def execute(command):
    string = OMPython.execute(command)
    error = OMPython.execute('''getErrorString()''').strip()[1:-1]
    re_error = re.compile('error',re.I)
    if re_error.match(error):
        raise Exception(error)
    return string

def run_example(example):
    re_package = re.compile('within[\s]+([\w.]+)')
    re_model = re.compile('model[\s]+([\w.]+)')
    package = None
    model = None
    with open(example) as f:
        lines = f.readlines();    
    for line in lines:
        result = re_package.match(line)
        if result:
            package = result.group(1)
        result = re_model.match(line)
        if result:
            model = result.group(1)    
    if not package:
        raise Exception("package not found in " + example)
    if not model:
        raise Exception("package not found in " + example)
    execute('''simulate(%s.%s)''' % (package,model))
