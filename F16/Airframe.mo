within F16;

model Airframe

  // control variables
  input Real throttle;
  input Real elevator;
  input Real aileron;
  input Real rudder;

  // output
  output Real an "north acceleration";
  output Real alat "east acceleration";
  output Real ax "not used";
  output Real qbar "dynamics pressure";
  output Real amach "mach number";

  // physical attributes
  parameter Real s "planform area";
  parameter Real b "wing span";
  parameter Real cbar "mean chord length";
  parameter Real weight "weight of aircraft";
  parameter Real Ixx "inertia about x axis @cm";
  parameter Real Iyy "inertia about y axis @cm";
  parameter Real Ixz "inertia xz term @cm";
  parameter Real hx "x-axis engine angular momentum";
  parameter Real xcg "center of gravity, % chord";
  parameter Real xcgr "aerodynamics reference point, % chord";
  parameter Real gd "gravitational acceleration";

  // precomputed inertia terms
  constant Real mass =weight/gd;
  constant Real Ixzs = Ixz^2;
  constant Real xpq = Ixz*(Ixx - Iyy + Izz);
  constant Real zeta = Ixx*Izz - Ixz^2;
  constant Real xqr = Izz*(Izz-Iyy) + Ixzs;
  constant Real zpq = (Ixx-Iyy)*Ixx  +Ixzs;
  constant Real ypr = Izz - Ixx;

  // states
  Real vt "true velocity";
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
  Real power "engine power";

  // temporary variables
  Real c_beta, u, v, w, s_theta, c_theta, s_phi, c_phi,
    s_psi, c_psi, qs, qsb, rmqs, g_c_theta, q_s_phi, ay,
    az, pq, qr, qhx, t1, t2, t3, s1, s2, s3, s4, s5, s6,
    s7, s8 , roll_m, pitch_m, yaw_m, dum, udot, vdot, wdot,
    tvt, b2v, cq;

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
  der(beta) = (vt*vdot - v*state_dot(1)) * c_beta / dum;

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

  // TODO: add atmos func
  der(power) = 0; // TODO: add power function

  // aerodynamics

  // precomputation for state equations

end Airframe;
