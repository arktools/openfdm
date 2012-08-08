#!/usr/bin/env python
# helps automate calling scripts via python
import os
import sys
from openfdm_path import root
import inspect
import OMPython

if len(sys.argv) == 2:
  shell = OMPython.OMShell(root,echo=True)
else:
  print "usage", sys.argv[0], "script", 
  sys.exit(1)

print shell.run_script(os.path.abspath(sys.argv[1]));

# vim:ts=2:sw=2:expandtab:
