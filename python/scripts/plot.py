#!/usr/bin/env python
# an example of plotting within python for openfdm
import sys
import os
from openfdm_path import root
from DyMat import DyMatFile
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.ticker import EngFormatter
import numpy as np

if len(sys.argv) != 2:
  print "usage", sys.argv[0], "Test_res.mat", 
  sys.exit(1)
fileName = sys.argv[1]
matFile = DyMatFile(os.path.abspath(fileName))
x = matFile.data('body.airframe.r_0[1]');
z = matFile.data('body.airframe.r_0[3]');

ax = plt.subplot(111)
formatter = EngFormatter(unit='m', places=1)
ax.xaxis.set_major_formatter(formatter)
ax.yaxis.set_major_formatter(formatter)
ax.plot(x, -z)
plt.xlabel('x')
plt.ylabel('-z')
plt.title('trajectory of ' + fileName)
plt.grid()
plt.show()

# vim:ts=2:sw=2:expandtab:
