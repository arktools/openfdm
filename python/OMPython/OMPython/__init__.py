# -*- coding: cp1252 -*-
"""
  This file is part of OpenModelica.

  Copyright (c) 1998-CurrentYear, Open Source Modelica Consortium (OSMC),
  c/o Linköpings universitet, Department of Computer and Information Science,
  SE-58183 Linköping, Sweden.

  All rights reserved.

  THIS PROGRAM IS PROVIDED UNDER THE TERMS OF GPL VERSION 3 LICENSE OR
  THIS OSMC PUBLIC LICENSE (OSMC-PL) VERSION 1.2.
  ANY USE, REPRODUCTION OR DISTRIBUTION OF THIS PROGRAM CONSTITUTES RECIPIENT'S ACCEPTANCE
  OF THE OSMC PUBLIC LICENSE OR THE GPL VERSION 3, ACCORDING TO RECIPIENTS CHOICE.

  The OpenModelica software and the Open Source Modelica
  Consortium (OSMC) Public License (OSMC-PL) are obtained
  from OSMC, either from the above address,
  from the URLs: http://www.ida.liu.se/projects/OpenModelica or
  http://www.openmodelica.org, and in the OpenModelica distribution.
  GNU version 3 is obtained from: http://www.gnu.org/copyleft/gpl.html.

  This program is distributed WITHOUT ANY WARRANTY; without
  even the implied warranty of  MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE, EXCEPT AS EXPRESSLY SET FORTH
  IN THE BY RECIPIENT SELECTED SUBSIDIARY LICENSE CONDITIONS OF OSMC-PL.

  See the full OSMC Public License conditions for more details.

  Author : Anand Kalaiarasi Ganeson, ganan642@student.liu.se, 2012-03-19
  Version: 1.0

  Modified by James Goppert, jgoppert@purdue.edu, 2012-08-04: class wrapper OMShell added
"""

import sys
import struct
import os
import time
import inspect
import re
import tempfile
import StringIO

from subprocess import Popen, PIPE
from datetime import datetime

# import the parser module
import OMParser

# import the skeletons for the global module
from omniORB import CORBA
from OMPythonIDL import _OMCIDL

# config file to set openmodelicahome during development
import OMConfig

class OMShell:

  def __init__(self,root_path,echo=False):

    # set enviornment to include root path
    self.root_path = root_path
    self.echo = echo
    os.environ['OPENMODELICALIBRARY'] = os.getenv('OPENMODELICALIBRARY') + ':' + root_path

    # regex's
    self.re_simulate = re.compile('.*simulate\(.*',re.M)
    self.re_error = re.compile('.*error|fail.*',re.I|re.M)
    self.re_package = re.compile('within[\s]+([\w.]+)')
    self.re_model = re.compile('model[\s]+([\w.]+)')

    # create build directory and move to it
    self.build_path = os.path.join(tempfile.gettempdir(),'ompython')
    if not os.path.isdir(self.build_path):
      os.makedirs(self.build_path)

    curdir = os.path.abspath(os.path.curdir)
    os.chdir(self.build_path)

    # Randomize the IOR file name
    random_string = str(datetime.now())
    random_string = ''.join(e for e in random_string if e.isalnum())

    # Create a log file in the temp directory
    temp = tempfile.gettempdir()
    omc_log_file = open(os.path.join(temp, "openmodelica.omc.output.OMPython"), 'w')

    # Look for the OMC
    try:
      omhome = os.environ['OPENMODELICAHOME']
      # add OPENMODELICAHOME\lib to PYTHONPATH so python can load omniORB libraries
      sys.path.append(os.path.join(omhome, 'lib'))
      sys.path.append(os.path.join(omhome, 'lib','python'))
      # add OPENMODELICAHOME\bin to path so python can find the omniORB binaries
      pathVar = os.getenv('PATH')
      pathVar += ';'
      pathVar += os.path.join(omhome, 'bin')
      os.putenv('PATH', pathVar)
      ompath = os.path.join(omhome, 'bin', 'omc') + " +d=interactiveCorba" + " +c=" + random_string
      server = Popen(ompath, shell=True, stdout=omc_log_file, stderr=omc_log_file)
    except:
      try:
        PREFIX = OMConfig.DEFAULT_OPENMODELICAHOME
        omhome = os.path.join(PREFIX)
        ompath = os.path.join(omhome, 'bin', 'omc') + " +d=interactiveCorba" + " +c=" + random_string
        server = Popen(ompath, shell=True, stdout=omc_log_file, stderr=omc_log_file)
      except:
        try:
          ompath = os.path.join('omc') + " +d=interactiveCorba" + " +c=" + random_string
          server = Popen(ompath, shell=True, stdout=omc_log_file, stderr=omc_log_file)
        except:
          "The OpenModelica compiler is missing in the System path, please install it"
    finally:
      os.chdir(curdir)

    # Locating and using the IOR
    if sys.platform == 'win32':
      ior_file = "openmodelica.objid." + random_string
    else:
      currentUser = os.environ['USER']
      if currentUser == '':
        currentUser = "nobody"
      ior_file = "openmodelica." + currentUser + ".objid." + random_string
    ior_file = os.path.join(temp, ior_file)
    omc_corba_uri= "file:///" + ior_file
    print "file: ", ior_file

    # See if the omc server is running
    if os.path.isfile(ior_file):
      pass
      print "OMC Server is up and running at " + omc_corba_uri + "\n"
    else:
      attempts = 0
      while True:
        if not os.path.isfile(ior_file):
          time.sleep(0.25)
          attempts +=1
          if attempts == 10:
            print "OMC Server is down. Please start it! Exiting...\n"
            sys.exit(2)
        else:
          print "OMC Server is up and running at " + omc_corba_uri + "\n"
          break

    #initialize the ORB with maximum size for the ORB set
    sys.argv.append("-ORBgiopMaxMsgSize")
    sys.argv.append("2147483647")
    orb = CORBA.ORB_init(sys.argv, CORBA.ORB_ID)

    # Read the IOR file
    objid_file=open(ior_file)
    ior = objid_file.readline()
    objid_file.close()

    # Find the root POA
    poa = orb.resolve_initial_references("RootPOA")

    # Convert the IOR into an object reference
    obj = orb.string_to_object(ior)

    # Narrow the reference to the OmcCommunication object
    omc = obj._narrow(_OMCIDL.OmcCommunication)

    # Check if we are using the right object
    if omc is None:
            print "Object reference is not valid"
            sys.exit(1)

    # same omc instance to class
    self.omc = omc

  def __del__(self):
    """ quit shell when deleted """
    self.omc.sendExpression("quit()")

  def get_build_path(self):
    return self.build_path;

  def executeMultiLine(self,commands):
    f = StringIO.StringIO(commands);
    result = None
    while True: # for all lines
      # get next line
      line = ''
      while True:
        data = f.read(1)
        if not data:
          return
        if data == ';':
          break
        line += str(struct.unpack("c", data)[0])
      yield self.execute(line)  
    
  # Invoke the sendExpression method to send text commands to the server
  def execute(self,command):
    if self.echo:
      print command
    curdir = os.path.abspath(os.path.curdir)
    os.chdir(self.build_path)
    if command == "quit()":
      self.omc.sendExpression("quit()")
      print "\nOMC has been Shutdown\n"
      os.chdir(curdir)
      sys.exit(1)
    else:
      result = self.omc.sendExpression(command)
      error = self.omc.sendExpression('getErrorString()').strip()[1:-1]
      try:
        if self.re_simulate.match(command):
          answer = OMParser.check_for_values(result)
          OMParser.result = {}
          if type(answer) is dict and answer != {}:
            result = answer
            messages = get(result,'SimulationResults.messages')
            print "messages: ", messages
            if self.re_error.match(messages):
              raise Exception(messages)
        elif type(result) is bool:
          if not (result):
            raise Exception("command: %s returned false" % command)
        elif type(result) is str:
          if self.re_error.match(result):
            raise Exception(result)

        if self.re_error.match(error):
          raise Exception(error)

      finally:
        os.chdir(curdir)

      return result

  # Test commmands
  def run(self):
    omc_running = True
    while omc_running:
      command = raw_input("\n>>")
      if command == "quit()":
        self.omc.sendExpression("quit()")
        print "\nOMC has been Shutdown\n"
        omc_running = False
        sys.exit(1)
      else:
        result = self.omc.sendExpression(command)
        answer = OMParser.check_for_values(result)
        OMParser.result = {}
        print answer

  def run_script(self,scriptFile):
    return self.execute('runScript("%s")' % scriptFile);

  def run_model(self,modelFile):
    packageName = None
    modelName = None
    with open(modelFile) as f:
      lines = f.readlines();    
    for line in lines:
      result = re_package.match(line)
      if result:
        packageName = result.group(1)
      result = re_model.match(line)
      if result:
        modelName = result.group(1)    
    if not packageName:
      raise Exception('package name not found in ' + modelFile)
    if not modelName:
      raise Exception('model name not found in ' + modelFile)
    return self.execute('simulate(%s.%s)' % (packageName,modelName))
  

def find_models(root_path):
  """find all models in a path"""
  for root, dirs, files in os.walk(root_path):
    for fileName in files:
      if fileName != "package.mo" and os.path.splitext(fileName)[1] == ".mo":
        yield os.path.join(root,fileName)

def find_scripts(root_path):
  """find all scripts in a path"""
  for root, dirs, files in os.walk(root_path):
    for fileName in files:
      if os.path.splitext(fileName)[1] == ".mos":
        yield os.path.join(root,fileName)

# Helper class to retrieve the results of (nested) dictionaries using dot separated queries
class dotdictify(dict):
  def __init__(self, value=None):
      if value is None:
          pass
      elif isinstance(value, dict):
          for key in value:
              self.__setitem__(key, value[key])
      else:
          raise TypeError, 'expected a dictionary, re-try'

  def __setitem__(self, key, value):
      if '.' in key:
          myKey, restOfKey = key.split('.', 1)
          target = self.setdefault(myKey, dotdictify())
          if not isinstance(target, dotdictify):
              raise KeyError, 'cannot set "%s" in "%s" (%s)' % (restOfKey, myKey, repr(target))
          target[restOfKey] = value
      else:
          if isinstance(value, dict) and not isinstance(value, dotdictify):
              value = dotdictify(value)
          dict.__setitem__(self, key, value)

  def __getitem__(self, key):
      if '.' not in key:
          return dict.__getitem__(self, key)
      myKey, restOfKey = key.split('.', 1)
      target = dict.__getitem__(self, myKey)
      if not isinstance(target, dotdictify):
          raise KeyError, 'cannot get "%s" in "%s" (%s)' % (restOfKey, myKey, repr(target))
      return target[restOfKey]

  def __contains__(self, key):
      if '.' not in key:
          return dict.__contains__(self, key)
      myKey, restOfKey = key.split('.', 1)
      target = dict.__getitem__(self, myKey)
      if not isinstance(target, dotdictify):
          return False
      return restOfKey in target

  def setdefault(self, key, default):
      if key not in self:
          self[key] = default
      return self[key]

  __setattr__ = __setitem__
  __getattr__ = __getitem__


def typeCast(string):
  if string.__class__ == dict:
    string = dict(string)
  elif string.__class__ == list:
    string = list(string)
  elif string.__class__ == float:
    string = float(string)
  elif string.__class__ == long:
    string = long(string)
  elif string.__class__ == bool:
    string = bool(string)
  elif string.__class__ == tuple:
    string = tuple(string)
  elif string.__class__ == complex:
    string = complex(string)
  elif string.__class__ == int:
    string = int(string)
  elif string.__class__ == file:
    string = file(string)
  elif string.__class__ == str:
    string = str(string)
  elif string.__class__ == None:
    string = None
  elif inspect.isclass(dotdictify):
    try:
      string = dict(string)
      if string.__class__ == dict:
        string = dict(string)
    except:
      try:
        string = list(string)
        if string.__class__ == list:
          string = list(string)
      except:
        try:
          string = float(string)
          if string.__class__ == float:
            string = float(string)
        except:
          try:
            string = long(string)
            if string.__class__ == long:
              string = long(string)
          except:
            try:
              string = None(string)
              if string.__class__ == None:
                string = None(string)
            except:
              try:
                string = tuple(string)
                if string.__class__ == tuple:
                  string = tuple(string)
              except:
                try:
                  string = complex(string)
                  if string.__class__ == complex:
                    string = complex(string)
                except:
                  try:
                    string = int(string)
                    if string.__class__ == int:
                      string = int(string)
                  except:
                    try:
                      string = file(string)
                      if string.__class__ == file:
                        string = file(string)
                    except:
                      try:
                        string = str(string)
                        if string.__class__ == str:
                          string = str(string)
                      except:
                        try:
                          string = bool(string)
                          if string.__class__ == bool:
                            string = bool(string)
                        except:
                          print "Unknown datatype :: %s"% string
  return string

def get(root,query):
  if isinstance(root,dict):
    root = dotdictify(root)

  try:
    result = root[query]
    result = typeCast(result)
    return result
  except KeyError:
    print "KeyError: Cannot GET the value, please check the syntax of your dotted notationed query"

def set(root,query,value):
  if isinstance(root,dict):
    root = dotdictify(root)

  try:
    root[query]=value
    result = typeCast(root)
    return result
  except KeyError:
    print "KeyError: Cannot SET the value, please check your dotted notationed query"

if __name__ == "__main__":
  run()

# vim:ts=2:sw=2:expandtab:
