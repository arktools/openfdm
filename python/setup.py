#!/usr/bin/env python

import multiprocessing,logging
from setuptools import setup

setup(
    name = 'PyOpenFDM',
    version = '0.0.1',
    author = 'James Goppert',
    author_email = 'james.goppert@gmail.com',
    description = ('A package for aerospace simulation in Modelica'),
    license = 'GPLv3',
    keywords = 'modelica dymola openmodelica mat fdm aerospace flight dynamic model',
    url = 'git@github.com:arktools/openfdm.git',
    packages = ['pyopenfdm',
                'pyopenfdm/deps/OMPython/OMPython',
                'pyopenfdm/deps/OMPython/OMPython.OMParser',
                'pyopenfdm/deps/OMPython/OMPythonIDL',
                'pyopenfdm/deps/OMPython/OMPythonIDL._OMCIDL',
                'pyopenfdm/deps/OMPython/OMPythonIDL._OMCIDL__POA',
                'pyopenfdm/deps/DyMat/DyMat',
                'pyopenfdm/deps/DyMat/DyMat.Export',
                'pyopenfdm/deps/DyMat/DyMat.Plot'],
    entry_points={
        'console_scripts': [
            'pyopenfdm-run = pyopenfdm.run:command_line',
            'pyopenfdm-plot = pyopenfdm.plot:command_line'
        ]},
    classifiers=[
        'Development Status :: 4 - Beta',
        'Environment :: Console',
        'Intended Audience :: Science/Research',
        'License :: OSI Approved :: GNU General Public License v3 (GPLv3)',
        'Operating System :: OS Independent',
        'Programming Language :: Python :: 2.7',
        'Topic :: Scientific/Engineering',
    ],
    install_requires=['matplotlib','scipy'],
    test_suite='nose.collector',
    tests_require=['nose'],
)
