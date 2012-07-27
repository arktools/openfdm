within JetPack;
package fun
  function DCM "Direction cosine matrix: Earth to body"
    input Real phi;
    input Real theta;
    input Real psi;
    output Real Reb[3,3];
  algorithm
    Reb[1,1]:=cos(psi) * cos(theta);
    Reb[1,2]:=sin(psi) * cos(theta);
    Reb[1,3]:=-sin(theta);
    Reb[2,1]:=-sin(psi) * cos(phi) + cos(psi) * sin(theta) * sin(phi);
    Reb[2,2]:=cos(psi) * cos(phi) + sin(psi) * sin(theta) * sin(phi);
    Reb[2,3]:=cos(theta) * sin(phi);
    Reb[3,1]:=sin(psi) * sin(phi) + cos(psi) * sin(theta) * cos(phi);
    Reb[3,2]:=-cos(psi) * sin(phi) + sin(phi) * sin(theta) * cos(phi);
    Reb[3,3]:=cos(theta) * cos(phi);
  end DCM;
  function Transpose "Transpose 3 by 3 matrix - rewrite later for n by n"
    input Real A[3,3];
    output Real B[3,3];
  algorithm
    B[1,1]:=A[1,1];
    B[1,2]:=A[2,1];
    B[1,3]:=A[3,1];
    B[2,1]:=A[1,2];
    B[2,2]:=A[2,2];
    B[2,3]:=A[3,2];
    B[3,1]:=A[1,3];
    B[3,2]:=A[2,3];
    B[3,3]:=A[3,3];
  end Transpose;
  function d2r
    import Modelica.Constants;
    input Real deg;
    output Real rad;
  algorithm
    rad:=deg / 180.0 * Constants.pi;
  end d2r;
  function r2d
    import Modelica.Constants;
    input Real rad;
    output Real deg;
  algorithm
    deg:=(rad * 180.0) / Constants.pi;
  end r2d;
end fun;

