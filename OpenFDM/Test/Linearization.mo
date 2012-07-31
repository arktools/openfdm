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
 constant Real[2] x0 ={1,2};
 constant Real[2] u0 ={1,2};
 Real[2] x(start=x0);
 input Real u[2] = u0;
 output Real y[2];
 constant Real A[2,2] = [1,2;2,0];
 constant Real B[2,2] = [0,1;1,0];
 constant Real C[2,2] = [1,2;2,1];
 constant Real D[2,2] = [1,2;0,1];
equation
 der(x) = A * x + B * u;
 y = C*x + D * u;
end Linearization;
