within OpenFDM.Test;

record PositionGeodetic
  import SI=Modelica.SIunits;
  SI.Angle l "longitude";
  SI.Angle phi "latitude";
  SI.Position h "altitude";
end PositionGeodetic;

function ECEFToGeodetic
  "Aircraft Control and Simulation, Lewis and Stevens, pg 39"
  import SI=Modelica.SIunits;
  import C=Modelica.Constants;
  input SI.Position[3] r_ECEF;
  input SI.Length a;
  input SI.Length b;
  input Real e;
  input Real hTol;
  output PositionGeodetic r_GEO;
protected
    Real t1, t2, N, t2_0, e2;
algorithm 
  N := a;
  r_GEO.h := 0;
  r_GEO.l := atan2(r_ECEF[2],r_ECEF[1]);
  t1 := sqrt(r_ECEF[1]^2 + r_ECEF[2]^2);
  t2 := N + r_GEO.h;
  e2 := e^2;

  if (t1 < 1) then // avoid singularity near poles
    r_GEO.h := abs(r_ECEF[3]) - b;
    if (r_ECEF[3] > 0) then
      r_GEO.phi := C.pi/2;
    else
      r_GEO.phi := -C.pi/2;
    end if;
  else // iterative approach everywhere else
    while true loop
      r_GEO.phi := atan(r_ECEF[3]/(t1*(1-N*e^2/t2)));
      N := a/sqrt(1-e2*sin(r_GEO.phi)^2);
      t2_0 := t2;
      t2 := t1/(cos(r_GEO.phi));
      if (abs(t2 - t2_0) < hTol) then
        break;
      end if;
    end while;
    r_GEO.h := t2 - N;
  end if;
end ECEFToGeodetic;

function GeodeticToECEF
  "Aircraft Control and Simulation, Lewis and Stevens, pg 38"
  import SI=Modelica.SIunits;
  import C=Modelica.Constants;
  input PositionGeodetic r_GEO;
  input SI.Length a;
  input Real e;
  output SI.Position[3] r_ECEF;

protected
  SI.Length N;
algorithm
  N := a/sqrt(1-e^2*sin(r_GEO.phi)^2);
  r_ECEF[1] := (N+r_GEO.h)*cos(r_GEO.phi)*cos(r_GEO.l);
  r_ECEF[2] := (N+r_GEO.h)*cos(r_GEO.phi)*sin(r_GEO.l);
  r_ECEF[3] := (N*(1-e^2)+r_GEO.h)*sin(r_GEO.phi);
end GeodeticToECEF;

function GravityWGS84ECEF
  "Aircraft Control and Simulation, Lewis and Stevens, pg 41"
  import SI=Modelica.SIunits;
  import Modelica.Math.Vectors.norm;
  input SI.Position[3] p;
  input Real GM;
  input Real a;
  input Real J2;
  output SI.Acceleration[3] g;
protected
  SI.Angle psi;
  Real t1, t2, sPsi2;
  SI.Length r;
algorithm
  r :=  norm(p);
  t1 := -GM/r^2;
  t2 := 1.5 * J2 * (a/r)^2;
  sPsi2 := sin(psi)^2;
  psi := asin(p[3]/r);
  g[1] := t1 * (1 + t2 * (1-5*sPsi2)) * p[1]/r;
  g[2] := t1 * (1 + t2 * (1-5*sPsi2)) * p[2]/r;
  g[3] := t1 * (1 + t2 * (3-5*sPsi2)) * p[3]/r;
end GravityWGS84ECEF;

model TestConversions
  import SI=Modelica.SIunits;
  inner World world;
  SI.Position[3] p_ECEF;
  SI.Acceleration[3] g;
  PositionGeodetic p_GEO;
algorithm
  p_ECEF := {0,world.a,0};
  p_GEO := OpenFDM.Test.ECEFToGeodetic(p_ECEF,
    world.a,world.b,world.e,world.hTol);
  p_ECEF := OpenFDM.Test.GeodeticToECEF(p_GEO,
    world.a,world.e);
  g := GravityWGS84ECEF(p_ECEF,
    world.GM,world.a,world.J2);
end TestConversions;

// vim:ts=2:sw=2:expandtab:
