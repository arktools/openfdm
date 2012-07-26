#!/usr/bin/env python
#
#    Filename: pyopenfdm.py: python script to improve OMPython script interface
#    Copyright (C) 2012  James Goppert <james.goppert@gmail.com>
#
#    This program is free software: you can redistribute it and/or modify
#    it under the terms of the GNU General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    This program is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU General Public License for more details.
#
#    You should have received a copy of the GNU General Public License
#    along with this program.  If not, see <http:#www.gnu.org/licenses/>.

import os
import sys
import inspect
import tempfile
import argparse
import logging
import re

# find root path or project
root_path = os.path.abspath(os.path.join(inspect.getfile(inspect.currentframe()),os.path.pardir))

# set enviornment to include project root
os.environ['OPENMODELICALIBRARY'] = os.getenv('OPENMODELICALIBRARY') + ':' + root_path

class OMInterface(object):

    def __init__(self,script,echo,quiet,terminal):
        self.script = script
        self.echo = echo
        self.quiet = quiet
        self.terminal = terminal
        self.process()

        # handle logging
        if self.quiet:
            log_level = logging.WARNING
        else:
            log_level = logging.INFO
        logging.basicConfig(format='%(message)s',level=log_level)


    @classmethod
    def from_argv(cls,argv):
        sys.argv = argv
        parser = argparse.ArgumentParser(description='Runs modelica script using python.')
        parser.add_argument('-s','--script',default=None, help='script to run')
        parser.add_argument('-e', '--echo', default=False, action='store_true', help='enable echo')
        parser.add_argument('-q', '--quiet', default=False, action='store_true', help='quiet mode')
        parser.add_argument('-t', '--terminal', default=False, action='store_true', help='enable terminal')
        args = parser.parse_args()
        if (args.script == None) and (not args.terminal):
            parser.print_help()
            raise SystemExit("Error: please specify terminal or script option")
        return cls(args.script,args.echo,args.quiet,args.terminal)

    def process(self):

        # check that script exists
        if self.script:
            script_path = os.path.abspath(self.script)
            if not os.path.isfile(script_path):
                raise IOError("script %s not found" % script_path)

        # create build directory and move to it
        build_path = os.path.join(tempfile.gettempdir(),'ompython')
        if not os.path.isdir(build_path):
            os.makedirs(build_path)
        os.chdir(build_path)

        import OMPython

        def execute(command):
            if self.echo:
                print command
            string = OMPython.execute(command)
            error = OMPython.execute('''getErrorString()''').strip()[1:-1]
            re_error = re.compile('error',re.I)
            if re_error.match(error):
                if self.terminal:
                    print error
                    OMPython.run()
                else:
                    raise Exception(error)
            return string

         # check paths
        logging.info('OPENMODELICALIBRARY: %s' % execute('''getEnvironmentVar("OPENMODELICALIBRARY")'''))
        logging.info('working directory: %s' % execute('''cd()'''))

        # run script
        if self.script:
            logging.info('script path: "%s"' % script_path)

            # get next line of script
            def get_next_line(stringio):
                line = []
                while True:
                    byte = stringio.read(1)
                    if not byte:
                        line = []
                        break
                    if byte == ';':
                        break
                    line.append(byte)
                return ''.join(line)

            # run script line by line checking for error
            with open(script_path,'r') as script_file:
                re_error = re.compile('error',re.I)
                while True:
                    line = get_next_line(script_file)
                    if not line:
                        logging.info('%s completed successfully' % script_path)
                        break
                    result = execute(line)
                    print result

        if self.terminal:
            OMPython.run()

if __name__ == "__main__":
    OMInterface.from_argv(sys.argv)
