#!/usr/bin/env python
import os, sys, inspect

root = os.path.realpath(os.path.abspath(os.path.join(
    os.path.split(inspect.getfile( inspect.currentframe() ))[0],
    os.path.pardir,os.path.pardir)))

for dep in ['python',
            os.path.join('python','deps','OMPython'),
            os.path.join('python','deps','DyMat')]:
    path = os.path.join(os.path.abspath(os.path.join(root,dep)))
    if path not in sys.path:
        sys.path.insert(0, path)
