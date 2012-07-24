#!/usr/bin/env python
#
#    Filename: test.py: runs all examples in OpenFDM
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

import unittest
import OMPython

class TestCase(unittest.TestCase):

    def setUp(self):
        pass

    def tearDwon(self):
        pass

    def testExamples(self):
        print OMPython.execute("help()")

if __name__ == "__main__":
    unittest.main() # run all tests
