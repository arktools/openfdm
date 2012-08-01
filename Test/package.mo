package Test

import SI=Modelica.SIunits;
import C=Modelica.Constants;

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
  SI.Length b = a(1-f) "semi-minor axis"; 
  SI.Length M "meridian radius of curvature";
  SI.Length N "prime vertical radius of curvature";
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
protected
  Real cThe, cPsi, cPhi;
  Real sThe, sPsi, sPhi;
  Real tThe;
  Real gamma;
algorithm
  if (abs(theta) - C.pi/2) >  gimbalLockTol then
    cThe := cos(theta);
  else
    cThe := epsilon;
  end if;
  cPhi := cos(phi);  
  cThe := cos(theta);
  cPsi := cos(psi);
  sPhi := sin(phi);  
  sThe := sin(theta);
  sPsi := sin(psi);
  tThe := tan(theta);
  gamma := Jx*Jz-Jxz^2;
equation
  // force equations
  der(u) = r*v - q*w - gD*sThe + F_b[1]/m;
  der(v) = -r*u + p*w + gD*sPhi*cThe + F_b[2]/m;
  der(w) = q*u - p*v + gD*cPhi*cThe + F_b[3]/m;

  // kinematics
  der(phi) = p + tThe*(q*sinPhi + r*cPhi);
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
end Kinematics6DofFlatEarth;

/*
model KinematicsFlatEarth "planet assumed to be inertial frame and flat"
  SI.Position r_n "position vector in navigaton frame";
  SI.AngularVelocity w_b "angular velocity of body w.r.t earth in body frame";
  Real C_nb[3,3] "rotation matrix from body to navigation frame";
  Real C_bn[3,3] "rotation matrix from navigation to body frame";
  Real H[3,3] "transforms body angular rates to euler angle rates";
  Real gimbalLockTol = 0.001 "tolerance for gimbal lock to avoid singularities";
  Real epsilon = 0.0001 "replace trig with epsilon during gimbal lock";
  Real M_b[3] "moment in body frame";
  Real F_b[3] "force in body frame";
protected
  Real cThe, cPsi, cPhi;
  Real sThe, sPsi, sPhi;
algorithm
  if (abs(theta) - C.pi/2) >  gimbalLockTol then
    cThe := cos(theta);
  else
    cThe := epsilon;
  end if;
  cPsi := cos(theta);
  cPhi := cos(phi);  
  sThe := sin(theta);
  sPsi := sin(theta);
  sPhi := sin(phi);  
  tThe := tan(theta);
equation
  C_nb = {{                  cThe*cPsi,                   cThe*sPsi,     -sThe},
          {-cPhi*sPsi + sPhi*sThe*cPsi,  cPhi*cPsi + sPhi*sThe*sPsi, sPhi*cThe},
          { sPhi*sPsi + cPhi*sThe*cPsi, -sPhi*cPsi + cPhi*sThe*sPsi, cPhi*cThe}};
  C_bn = transpose(C_nb);
  H = {{1, tThe*sPhi, tThe*cPhi},
       {0, cPhi     , -sPhi    },
       {0, sPhi/cThe, cPhi/cThe}};
  der(r_n) = C_nb*v_b;
  der(phi_n) = H*w_b;
  der(v_b) = (1/m)*F_b + C_bn*g_n - cross(w_b,v_b);
  J*der(w_b) = M_b - cross(w_b,J*w_b);
end KinematicsFlatEarth;
*/

end Test;

// vim:ts=2:sw=2:expandtab
