#!/usr/bin/env python
# helps automate calling scripts via python
import os
import sys
import inspect
from deps import dep_root
import OMPython

def command_line():
  import argparse

  argparser = argparse.ArgumentParser()
  argparser.add_argument("root",
      help="the root of the modelica project")
  argparser.add_argument("script",
      help="the script to run")
  args = argparser.parse_args()

  shell = OMPython.OMShell(os.path.abspath(args.root),echo=True)

  print shell.run_script(os.path.abspath(args.script));

if __name__ == "__main__":
  command_line()

# vim:ts=2:sw=2:expandtab:
