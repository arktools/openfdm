package Test

import SI=Modelica.SIunits;
import C=Modelica.Constants;
import Modelica.Math;

function T1
  input Real a;
  output Real T[3,3];
algorithm
  T := {{  1,      0,      0},
        {  0, cos(a), sin(a)},
        {  0,-sin(a), cos(a)}};
annotation(Inline=true);
end T1;

function T2
  input Real a;
  output Real T[3,3];
algorithm
  T := {{ cos(a),  0,-sin(a)},
        {  0,      1,      0},
        { sin(a),  0, cos(a)}};
annotation(Inline=true);
end T2;

function T3
  input Real a;
  output Real T[3,3];
algorithm
  T := {{ cos(a), sin(a), 0},
        {-sin(a), cos(a), 0},
        {      0,      0, 1}};
annotation(Inline=true);
end T3;

model World
  parameter SI.Acceleration gD = 9.80665;
  parameter SI.Density rho0 = 1;
  parameter SI.Length a = 6378137.0 "semi-major axis";
  parameter Real f = 1/298.257223563 "(a-b)/a";
  Real e = sqrt(a^2-b^2)/a "eccentricity";
  Real e2 = e^2;
  SI.Length b = a*(1-f) "semi-minor axis"; 

  function M = MBase(a=a,e=e);
  function N = NBase(a=a,e=e);
  function rho = rhoBase(rho0=rho0);
  function g_n = g_nBase(gD=gD);
  function wind_n = wind_nBase();
  function ground = groundBase();

protected

  function MBase "meridian radius of curvature"
    input SI.Angle lat;
    input SI.Length a;
    input Real e;
    output SI.Length M;
  algorithm  
    M := a*(1-e^2)/(1-e^2*sin(lat)^2)^(3/2);
  annotation(Inline=true);
  end MBase;

  function NBase "prime vertical radius of curvature"
    input SI.Angle lat;
    input SI.Length a;
    input Real e;
    output SI.Length N;
  algorithm  
    N := a/sqrt(1-e^2*sin(lat)^2);
  annotation(Inline=true);
  end NBase;

  function rhoBase "air density"
    input SI.Position asl;
    input SI.Density rho0;
    output SI.Density rho;
  algorithm
    rho := rho0;
  annotation(Inline=true);
  end rhoBase;

  function g_nBase "gravity vector in nav frame"
    input SI.Position asl;
    input SI.Acceleration gD;
    output SI.Acceleration[3] g_n;
  algorithm
    g_n := {0,0,gD};
  annotation(Inline=true);
  end g_nBase;

  function wind_nBase "wind vector in nav frame"
    input SI.Angle lat;
    input SI.Angle lng;
    input SI.Position asl;
    output SI.Velocity[3] wind_n;
  algorithm
    wind_n := {0,0,0};
  annotation(Inline=true);
  end wind_nBase;

  function groundBase "wind vector in nav frame"
    input SI.Angle lat;
    input SI.Angle lng;
    output SI.Position agl;
  algorithm
    agl := 0;
  annotation(Inline=true);
  end groundBase;

end World;

model Aircraft6DofFlatEarth
  parameter SI.Mass m = 1 "mass";
  parameter SI.MomentOfInertia J[3,3] = identity(3);
  outer World world;
  Real C_nb[3,3];
  Real C_bn[3,3];
  Real eulerRates_wb[3,3];
  SI.Velocity[3] v_b "velocity in body frame";
  SI.Acceleration[3] a_b "acceleration in body frame";
  SI.AngularVelocity[3] w_b "angular velocity in body frame";
  SI.Force F_b[3] "force in body frame";
  SI.Torque M_b[3] "moment in body frame";
  SI.Angle phi "euler roll angle";
  SI.Angle theta "euler pitch angle";
  SI.Angle psi "euler heading angle";
  SI.Velocity v_n[3] "velocity in NED frame";
  SI.Position asl "altitude above sea level";
  SI.Angle lat "latitude";
  SI.Angle lng "longitude";
  SI.AngularMomentum H[3] "angular momentum";

// aerodynamic properties
  SI.Velocity vt "true airspeed";
  SI.Velocity vR_b[3] "relative velocity in body frame";
  //SI.Acceleration aR_b[3] "relative acceleration in body frame";
  
  SI.Velocity vtTol = 0.01 "velocity at which to ignore aerodynamics";
  SI.Acceleration vtDot "Derivative of true airspeed";
  SI.Angle alpha "angle of attack";
  SI.AngularVelocity alphaDot "angle of attack derivative";
  SI.Angle beta "side slip angle";
  SI.Angle betaDot "side slip angle derivative";
  SI.Pressure qBar "average dynamics pressure";

equation

  // force and moment
  F_b = C_bn*world.g_n(asl);
  M_b = {0,0,0};

  // rotatoin matrices
  C_bn = T1(phi)*T2(theta)*T3(psi); // body 3-2-1
  C_nb = transpose(C_bn);
  eulerRates_wb  = 
    {{1, tan(theta)*sin(phi), tan(theta)*cos(phi)},
     {0,            cos(phi),           -sin(phi)},
     {0, sin(phi)/cos(theta), cos(phi)/cos(theta)}};

  // dynamics
  H = J*w_b;
  der(H) + cross(w_b,H) = M_b;  
  v_n = C_nb*v_b;
  der(v_b) = a_b;
  a_b + cross(w_b,v_b) = F_b/m;
  der({phi,theta,psi}) = eulerRates_wb*w_b;

  // navigation equations
  der(lat) = v_n[1]/(world.M(lat)+asl);
  der(lng) = v_n[2]/(world.N(lat)+asl)/cos(lat);
  der(asl) = -v_n[3];

  // aerodynamics
  vR_b = C_bn*(v_n - world.wind_n(lat,lng,asl));
  qBar = 0.5*world.rho(asl)*vt^2;

  // avoid singularity in side slip angle/ vt
  // when magnitude of vt is negligible
  if (sqrt(vR_b*vR_b) > vtTol) then
    vt = sqrt(vR_b*vR_b);
    beta = asin(vR_b[2]/vt);
    betaDot = der(beta);
    vtDot = der(vt);
  else
    vt = 0;
    beta = 0;
    betaDot = 0;
    vtDot = 0;
  end if;

  // avoid singularity in alpha when relative
  // forward velocity is zero
  if ( abs(vR_b[1]) > vtTol) then
    alpha = atan2(vR_b[3],vR_b[1]);
    alphaDot = der(alpha);
  else
    alpha = 0;
    alphaDot = 0;
  end if;

end Aircraft6DofFlatEarth;

model Test1
  inner World world;
  Aircraft6DofFlatEarth aircraft;
end Test1;

end Test;

// vim:ts=2:sw=2:expandtab
