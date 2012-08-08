#!/usr/bin/env python
# generate time till failure plot
import os
import sys
import inspect
from openfdm_path import root
import OpenFDM
from OMPython import OMShell, get
from DyMat import DyMatFile

# plotting
from mpl_toolkits.mplot3d import Axes3D
import matplotlib.pyplot as plt
from matplotlib.ticker import EngFormatter
import numpy as np

if not os.path.isdir('fig'):
  os.mkdir('fig')

shell = OMShell(root, echo=True)
for item in shell.executeMultiLine(
"""
  loadModel(Modelica);
  loadModel(OpenFDM);
  loadModel(test);
"""):
    print " ", item

for i in xrange(10):

  print shell.execute("""loadString("
    model Rocket
      import OpenFDM.*;
      inner World world;
      Parts.RigidReferencePoint p(
        r_r(start={0,0,-1},fixed=true),
        euler(start={0,1,0},fixed=true));
      Parts.RigidBody structure(I_b=identity(3),m=0.1);
      Propulsion.SolidRocketMotor motor(
        mInert=0.1,
        mFuel=0.2,
        Ve=1000,
        mDot=1.0);
      Parts.RigidLink_B321 t(angles={0,0,%f/10},r_a={%f/10,0,0});
    equation
      connect(p.fA,structure.fA);
      connect(structure.fA,t.fA);
      connect(t.fB,motor.fA);
    end Rocket;
  ")""" % (i,i) )

  result = shell.execute("simulate(Rocket,stopTime=100,numberOfIntervals=10000)")
  resultFile = get(result, "SimulationResults.resultFile")[1:-1]
  resultFile = os.path.splitext(resultFile)[0]

  matFile = DyMatFile(resultFile)
  time = matFile.abscissa('p.r_r[1]', valuesOnly=True)
  print "time end", time[-1]
  x = matFile.data('p.r_r[1]')
  y = matFile.data('p.r_r[2]')
  agl = matFile.data('p.agl')

  formatter = EngFormatter(unit='m', places=1)

  fig1 = plt.figure()
  a1 = fig1.add_subplot(111)
  a1.set_title('trajectory of Rocket %d' % i)
  a1.set_xlabel('x')
  a1.set_ylabel('agl')
  a1.xaxis.set_major_formatter(formatter)
  a1.yaxis.set_major_formatter(formatter)
  a1.plot(x, agl)
  a1.grid()
  plt.savefig('fig/rocket-2d-%s.pdf'% i)


  fig2 = plt.figure()
  a2 = fig2.add_subplot(111, projection='3d')
  a2.set_title('trajectory of Rocket')
  a2.set_xlabel('x')
  a2.set_ylabel('y')
  a2.set_zlabel('agl')
  a2.xaxis.set_major_formatter(formatter)
  a2.yaxis.set_major_formatter(formatter)
  a2.zaxis.set_major_formatter(formatter)
  a2.plot(x, y, agl)
  a2.grid()
  plt.savefig('fig/rocket-3d-%s.pdf'% i)

# vim:ts=2:sw=2:expandtab:
