package Test

import SI=Modelica.SIunits;
import C=Modelica.Constants;

model Test1
  Real a;
equation
  der(a) = 1;
end Test1;

model Kinematics6DofFlatEarth
  parameter SI.Mass m = 1 "mass";
  parameter Real Jx = 1;
  parameter Real Jy = 1;
  parameter Real Jz = 1;
  parameter Real Jxz = 0;
  parameter SI.Acceleration gD = 9.80665;
  parameter SI.Length a = 6378137.0 "semi-major axis";
  parameter Real f = 1/298.257223563 "(a-b)/a";
  parameter Real gimbalLockTol = 0.001;
  parameter Real epsilon = 0.001;

  Real e = sqrt(a^2-b^2)/a "eccentricity";
  Real e2 = e^2;
  SI.Length b = a*(1-f) "semi-minor axis"; 
  SI.Length M "meridian radius of curvature";
  SI.Length N "prime vertical radius of curvature";
  SI.Velocity u "body x velocity";
  SI.Velocity v "body y velocity";
  SI.Velocity w "body z velocity";
  SI.Force F_b[3] "force in body frame";
  SI.Torque M_b[3] "moment in body frame";
  SI.AngularVelocity p "roll rate";
  SI.AngularVelocity q "pitch rate";
  SI.AngularVelocity r "yaw rate";
  SI.Angle phi "euler roll angle";
  SI.Angle theta "euler pitch angle";
  SI.Angle psi "euler heading angle";
  SI.Velocity v_n[3] "velocity in NED frame";
  SI.Position asl "altitude above sea level";
  SI.Angle lat "latitude";
  SI.Angle lng "longitude";

// aerodynamic properties
  /*SI.Velocity vt "true airspeed";*/
  /*SI.Acceleration vtDot "Derivative of true airspeed";*/
  /*SI.Angle alpha "angle of attack";*/
  /*SI.AngularVelocity alphaDot "angle of attack derivative";*/
  /*SI.Angle beta "side slip angle";*/
  /*SI.Angle betaDot "side slip angle derivative";*/
  /*SI.Pressure qBar "average dynamics pressure";*/

protected
  Real cThe, cPsi, cPhi;
  Real sThe, sPsi, sPhi;
  Real tThe;
  Real gamma;
algorithm
  cPhi := cos(phi);  
  cThe := cos(theta);
  cPsi := cos(psi);
  sPhi := sin(phi);  
  sThe := sin(theta);
  sPsi := sin(psi);
  tThe := tan(theta);
  gamma := Jx*Jz-Jxz^2;
  if (abs(theta) - C.pi/2) <  gimbalLockTol then
    cThe := epsilon;
  end if;
equation
  F_b = {1,1,1};
  M_b = {1,1,1};
  // force equations
  der(u) = r*v - q*w - gD*sThe + F_b[1]/m;
  der(v) = -r*u + p*w + gD*sPhi*cThe + F_b[2]/m;
  der(w) = q*u - p*v + gD*cPhi*cThe + F_b[3]/m;

  // kinematics
  der(phi) = p + tThe*(q*sPhi + r*cPhi);
  der(theta) = q*cPhi - r*sPhi;
  der(psi) = (q*sPhi + r*cPhi)/cThe;

  // moment equations
  gamma * der(p) = Jxz*(Jx - Jy + Jz)*p*q - (Jz*(Jz-Jy) + Jxz^2)*q*r + Jz*M_b[1] + Jxz*M_b[3];
  Jy*der(q) = (Jz - Jx)*p*r - Jz*(p^2 - r^2) + M_b[2];
  gamma * der(r) = ((Jx - Jy)*Jx + Jxz^2)*p*q - Jxz*(Jx - Jy + Jz)*q*r + Jxz*M_b[1] + Jx*M_b[3];

  // navigation equations
  M = a*(1-e2)/(1-e2*sin(lat)^2)^(3/2);
  N = a/(1-e2*sin(lat)^2)^(1/2);
  v_n[1] = u*cThe*cPsi + v*(-cPhi*sPsi + sPhi*sThe*cPsi) + w*(sPhi*sPsi + cPhi*sThe*cPsi);
  v_n[2] = u*cThe*sPsi + v*(cPhi*cPsi + sPhi*sThe*sPsi) + w*(-sPhi*cPsi + cPhi*sThe*sPsi);
  v_n[3] = u*sThe - v*sPhi*cThe - w*cPhi*cThe;

  der(lat) = v_n[1]/(M+asl);
  der(lng) = v_n[2]/(N+asl)/cPhi;
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

end Kinematics6DofFlatEarth;

end Test;

// vim:ts=2:sw=2:expandtab
