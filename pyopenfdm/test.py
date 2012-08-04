import os
from nose.tools import with_setup

from pyopenfdm import *

class Test(object):

    test_models_enabled = False # model testing not very useful

    def __init__(self):
        pass

    def __del__(self):
        om_shell('quit()')

    def test_scripts(self):
        for script in find_scripts(os.path.join(root_path,'test')):
            yield run_script, script

    def test_models(self):
        if self.test_models_enabled:
            for model in find_models(os.path.join(root_path,'test')):
                yield run_model, model
