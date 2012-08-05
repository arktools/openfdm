import os
import inspect
from nose.tools import with_setup
import unittest

from OMPython import OMShell, find_scripts, find_models

# find root path or project
class Test(unittest.TestCase):

  def setUp(self):

    # find root path of project
    self.root_path = os.path.abspath(os.path.join(
      inspect.getfile(inspect.currentframe()),
      os.path.pardir,os.path.pardir,os.path.pardir))

    # start shell
    self.shell = OMShell(self.root_path)

  def tearDown(self):
    pass

  def test_scripts(self):
    for script in find_scripts(os.path.join(self.root_path,'test')):
      self.shell.run_script(script);

  #def test_models(self):
    #for model in find_models(os.path.join(self.root_path,'test')):
      #yield self.shell.run_model, model

# vim:ts=2:sw=2:expandtab:
