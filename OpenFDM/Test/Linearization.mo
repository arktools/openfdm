//    Linearization: simple model for debugging
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

within OpenFDM.Test;

model Linearization
 Real x1(start=1);
 Real x2(start=2);
 parameter Real a=6,b=2,c=4;
 input Real u = 0;
 output Real y;
equation
 der(x1) = x1*(a-b*x1-x2);
 der(x2) = x2*(c-x1-x2) + u;
 y = x1 * u + x2 * u;
end Linearization;
