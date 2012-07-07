//    TestModel: simple model for debugging
//
//    Copyright (C) 2012  James Goppert <james.goppert@gmail.com>
//
//    This program is free software: you can redistribute it and/or modify
//    it under the terms of the GNU General Public License as published by
//    the Free Software Foundation, either version 3 of the License, or
//    (at your option) any later version.
//
//    This program is distributed in the hope that it will be useful,
//    but WITHOUT ANY WARRANTY; without even the implied warranty of
//    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//    GNU General Public License for more details.
//
//    You should have received a copy of the GNU General Public License
//    along with this program.  If not, see <http://www.gnu.org/licenses/>.

within OpenFDM.Examples;

model TestModel
  
  Kinematics.FlatEarthBody6DOF body(m=1,Jx=1,Jy=1,Jz=1,Jxz=0);
  //Modelica.Mechanics.MultiBody.World world;
  input Real a(start = 0);
  output Real b(start = 0);
  
equation
  
  //connect(world.frame_b,body.XYZ);
  der(a) = 1;
  der(b) = a;
  
end TestModel;
