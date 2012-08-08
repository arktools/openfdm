import os
import inspect
from nose.tools import with_setup
import unittest

from OMPython import OMShell, find_scripts, find_models

# find root path of project
root_path = os.path.abspath(os.path.join(
  inspect.getfile(inspect.currentframe()),
  os.path.pardir,os.path.pardir,os.path.pardir))

# start shell
shell = OMShell(root_path)

def test_scripts():
  for script in find_scripts(os.path.join(root_path,'test')):
    yield shell.run_script, script


#def test_models(self):
  #for model in find_models(os.path.join(self.root_path,'test')):
    #yield self.shell.run_model, model

# vim:ts=2:sw=2:expandtab:
