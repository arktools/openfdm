#!/usr/bin/env python

import os
import multiprocessing,logging
from setuptools import setup

def read(fname):
    return open(os.path.join(os.path.dirname(__file__), fname)).read()

setup(
    name = "OpenFDM",
    version = "0.0",
    author = "James Goppert",
    author_email = "james.goppert@gmail.com",
    description = ("a package for aerospace simulation in Modelica"),
    license = "GPLv3",
    keywords = "modelica dymola openmodelica mat fdm aerospace flight dynamic model",
    url = "git@github.com:arktools/openfdm.git",
    packages = ['OpenFDM',
                'OpenFDM/deps/OMPython/OMPython',
                'OpenFDM/deps/OMPython/OMPython.OMParser',
                'OpenFDM/deps/OMPython/OMPythonIDL',
                'OpenFDM/deps/OMPython/OMPythonIDL._OMCIDL',
                'OpenFDM/deps/OMPython/OMPythonIDL._OMCIDL__POA',
                'OpenFDM/deps/DyMat/DyMat',
                'OpenFDM/deps/DyMat/DyMat.Export',
                'OpenFDM/deps/DyMat/DyMat.Plot'],
    scripts = ['scripts/ttf.py'],
    long_description = read('../README.md'),
    classifiers = [
        "Development Status :: 4 - Beta",
        "Environment :: Console",
        "License :: OSI Approved :: GPLv3 License",
        "Programming Language :: Python :: 2.7",
        "Intended Audience :: Science/Research",
        "Topic :: Scientific/Engineering",
        "Topic :: Utilities"
    ],
    install_requires=['matplotlib','scipy'],
    test_suite="nose.collector",
    tests_require=['nose']
)
