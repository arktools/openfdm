within F16;

model Airframe

  // components
  input Real thrust;

  // atmosphere
  input Real qbar "dynamics pressure";
  input Real mach "mach number";

  // control variables
  input Real throttle;
  input Real elevator;
  input Real aileron;
  input Real rudder;

  // aerodynamic inputs
  input Real cx "force x aero coef";
  input Real cy "force y aero coef";
  input Real cz "force z aero coef";
  input Real cl "roll moment aero coef";
  input Real cm "pitch moment aero coef";
  input Real cn "yaw moment aero coef";

  // output
  output Real an "north acceleration";
  output Real alat "east acceleration";
  //output Real ax "not used";

  // physical attributes
  parameter Real s(start=100) "planform area";
  parameter Real b(start=10) "wing span";
  parameter Real cbar(start=1) "mean chord length";
  parameter Real weight(start=1) "weight of aircraft";
  parameter Real Ixx(start=1) "inertia about x axis @cm";
  parameter Real Iyy(start=1) "inertia about y axis @cm";
  parameter Real Izz(start=1) "inertia about z axis @cm";
  parameter Real Ixz(start=0) "inertia xz term @cm";
  parameter Real hx(start=1) "x-axis engine angular momentum";
  parameter Real xcg(start=0.3) "center of gravity, % chord";
  parameter Real xcgr(start=0.4) "aerodynamics reference point, % chord";
  parameter Real gd(start=9.8) "gravitational acceleration";

  // precomputed inertia terms
  Real mass = weight/gd;
  Real Ixzs = Ixz^2;
  Real xpq = Ixz*(Ixx - Iyy + Izz);
  Real zeta = Ixx*Izz - Ixz^2;
  Real xqr = Izz*(Izz-Iyy) + Ixzs;
  Real zpq = (Ixx-Iyy)*Ixx  +Ixzs;
  Real ypr = Izz - Ixx;

  // states
  Real vt(start=1) "true velocity";
  Real alpha "angle of attack";
  Real beta "side slip angle";
  Real phi "roll angle";
  Real theta "pitch angle";
  Real psi "heading angle";
  Real p "body roll rate";
  Real q "body pitch rate";
  Real r "body yaw rate";
  Real posNorth "north position";
  Real posEast "east position";
  Real alt "altitude";

protected

  // temporary variables
  Real c_beta, u, v, w, s_theta, c_theta, s_phi, c_phi,
    s_psi, c_psi, qs, qsb, rmqs, g_c_theta, q_s_phi, ay,
    az, udot, vdot, wdot, dum, pq, qr, qhx, roll_m, pitch_m, 
    yaw_m, t1, t2, t3, s1, s2, s3, s4, s5, s6, s7, s8; 

equation

  // precompute state equations
  c_beta = cos(beta);
  u = vt*cos(alpha)*c_beta;
  v = vt*sin(beta);
  w = vt*sin(alpha)*c_beta;
  s_theta = sin(theta);
  c_theta = cos(theta);
  s_phi = sin(phi);
  c_phi = cos(phi);
  s_psi = sin(psi);
  c_psi = cos(psi);
  qs = qbar*s;
  qsb = qs * b;
  rmqs = qs/mass;
  g_c_theta = gd * c_theta;
  q_s_phi = q * s_phi;
  ay = rmqs * cy;
  az = rmqs * cz;

  // force equation
  udot = r*v - q*w - gd * s_theta + (qs * cx + thrust)/mass;
  vdot = p*w - r*u + g_c_theta * s_phi + ay;
  wdot = q*u - p*v + g_c_theta * c_phi + az;
  dum = u*u + w*w;
  der(vt) = (u*udot + v*vdot + w*wdot)/vt;
  der(alpha) = (u*wdot - w*udot) / dum;
  der(beta) = (vt*vdot - v*der(vt)) * c_beta / dum;

  // kinematics
  der(phi) = p + (s_theta/c_theta)*(q_s_phi + r*c_phi);
  der(theta) = q*c_phi - r*s_phi;
  der(psi) = (q_s_phi + r*c_phi)/c_theta;

  // moments
  roll_m = qsb*cl;
  pitch_m = qs * cbar * cm;
  yaw_m = qsb*cn;
  pq = p*q;
  qr = q*r;
  qhx = q*hx;
  der(p) = ( xpq*pq - xqr*qr + Izz*roll_m + Ixz*(yaw_m + qhx) ) /zeta;
  der(q) = ( ypr*p*r - Ixz*(p^2 - r^2) + pitch_m - r*hx ) / Iyy;
  der(r) = ( zpq*pq - xpq*qr + Ixz*roll_m + Ixx*(yaw_m + qhx) ) /zeta;

  // navigation
  t1 = s_phi * c_psi;
  t2 = c_phi * s_theta;
  t3 = s_phi * s_psi;
  s1 = c_theta * c_psi;
  s2 = c_theta * s_psi;
  s3 = t1 * s_theta - c_phi * s_psi;
  s4 = t3 * s_theta + c_phi * c_psi;
  s5 = s_phi * c_theta;
  s6 = t2*c_phi + t3;
  s7 = t2 * s_psi - t1;
  s8 = c_phi * c_theta;
  der(posNorth) = u * s1 + v * s3 + w * s6;
  der(posEast) = u * s2 + v * s4 + w * s7;
  der(alt) = u * s_theta - v * s5 - w * s8;

  // outputs
  an = -az/gd;
  alat = ay/gd;

end Airframe;

// vim:ts=2:sw=2:expandtab:
