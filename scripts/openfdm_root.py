# generate time till failure plot
import os
import sys
import inspect

root=os.path.realpath(os.path.abspath(os.path.join(
  os.path.split(inspect.getfile( inspect.currentframe() ))[0],
  os.path.pardir)))
sys.path.insert(0,root)

