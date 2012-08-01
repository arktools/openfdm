package Test

import SI=Modelica.SIunits;
import C=Modelica.Constants;

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

model Aircraft6DofFlatEarth
  parameter SI.Mass m = 1 "mass";
  parameter Real J[3,3] = identity(3);
  parameter SI.Acceleration gD = 9.80665;
  parameter SI.Length a = 6378137.0 "semi-major axis";
  parameter Real f = 1/298.257223563 "(a-b)/a";
  Real C_nb[3,3];
  Real C_bn[3,3];
  Real eulerRates_wb[3,3];
  Real g_n[3];

  Real e = sqrt(a^2-b^2)/a "eccentricity";
  Real e2 = e^2;
  SI.Length b = a*(1-f) "semi-minor axis"; 
  SI.Length M "meridian radius of curvature";
  SI.Length N "prime vertical radius of curvature";
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
  /*SI.Velocity vt "true airspeed";*/
  /*SI.Acceleration vtDot "Derivative of true airspeed";*/
  /*SI.Angle alpha "angle of attack";*/
  /*SI.AngularVelocity alphaDot "angle of attack derivative";*/
  /*SI.Angle beta "side slip angle";*/
  /*SI.Angle betaDot "side slip angle derivative";*/
  /*SI.Pressure qBar "average dynamics pressure";*/

equation

  // gravity model
  g_n = {0,0,gD};

  // rotatoin matrices
  C_bn = T1(phi)*T2(theta)*T3(psi); // body 3-2-1
  C_nb = transpose(C_bn);
  eulerRates_wb  = 
    {{1, tan(theta)*sin(phi), tan(theta)*cos(phi)},
     {0,            cos(phi),           -sin(phi)},
     {0, sin(phi)/cos(theta), cos(phi)/cos(theta)}};

  // force and moment
  F_b = {1,1,1};
  M_b = {1,1,1};

  // dynamics
  H = J*w_b;
  der(H) + cross(w_b,H) = M_b;  
  v_n = C_nb*v_b;
  der(v_b) = a_b;
  a_b + cross(w_b,v_b) = F_b/m + C_bn*g_n;
  der({phi,theta,psi}) = eulerRates_wb*w_b;

  // navigation equations
  M = a*(1-e2)/(1-e2*sin(lat)^2)^(3/2);
  N = a/(1-e2*sin(lat)^2)^(1/2);
  der(lat) = v_n[1]/(M+asl);
  der(lng) = v_n[2]/(N+asl)/cos(phi);
  der(asl) = -v_n[3];

  // aerodynamics
  /*a_0 = der(v_0);*/
  /*vR_n = v_0 - env.wind_NED;*/
  /*aR_n = a_0; // TODO: - der(env.wind_NED);*/
  /*vR_b = resolve2(frame_b.R,vR_n);*/
  /*aR_b = resolve2(frame_b.R,aR_n);*/
  /*vt = Vectors.norm(vR_b);*/
  /*{p,q,r} = angularVelocity2(frame_b.R);*/

  /*alpha = atan2(vR_b[3],vR_b[1]);*/
  /*qBar = 0.5*env.rho*vt^2;*/

  /*// avoid singularity in side slip angle calc*/
  /*if (vt > vtTol) then*/
    /*beta = asin(vR_b[2]/vt);*/
    /*betaDot = (aR_b[2]*vt - aR_b[2]*vtDot)/vt*sqrt(vR_b[1]^2 + vR_b[3]^2);*/
    /*vtDot = (vR_b[1]*aR_b[1] + */
      /*vR_b[2]*aR_b[2] +*/
      /*vR_b[3]*aR_b[3])/vt;*/
  /*else*/
    /*beta = 0;*/
    /*betaDot = 0;*/
    /*vtDot = 0;*/
  /*end if;*/

  /*// if negligible airspeed, set wind angles to zero*/
  /*// to avoid singularity*/
  /*if ( (vR_b[1]^2 + vR_b[3]^2) > vtTol) then*/
    /*alphaDot = (vR_b[1]*aR_b[3]-vR_b[3]*aR_b[1])/(vR_b[1]^2 + vR_b[3]^2); //stevens & lewis pg 78*/
  /*else*/
    /*alphaDot = 0;*/
  /*end if;*/

end Aircraft6DofFlatEarth;

end Test;

// vim:ts=2:sw=2:expandtab
