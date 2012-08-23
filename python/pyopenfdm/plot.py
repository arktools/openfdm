#!/usr/bin/env python
# an example of plotting within python for openfdm
import os
import sys
import inspect
from deps import dep_root
from DyMat import DyMatFile
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.ticker import EngFormatter
import numpy as np

def command_line():
  import argparse

  argparser = argparse.ArgumentParser()
  argparser.add_argument("result",
      help="the result.mat file")
  args = argparser.parse_args()

  matFile = DyMatFile(os.path.abspath(args.result))
  x = matFile.data('p.r_r[1]');
  z = matFile.data('p.r_r[3]');

  ax = plt.subplot(111)
  formatter = EngFormatter(unit='m', places=1)
  ax.xaxis.set_major_formatter(formatter)
  ax.yaxis.set_major_formatter(formatter)
  ax.plot(x, -z)
  plt.xlabel('x')
  plt.ylabel('-z')
  plt.title('trajectory of ' + args.result)
  plt.grid()
  plt.show()

if __name__ == "__main__":
  command_line()

# vim:ts=2:sw=2:expandtab:
