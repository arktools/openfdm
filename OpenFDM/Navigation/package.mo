within OpenFDM;

package Navigation

function quatToDcm
  input Real q[4];
  output Real C_nb[3,3];
protected
  Real a,b,c,d;
algorithm
  a := q[1];
  b := q[2];
  c := q[3];
  d := q[4];
  C_nb := {{ a^2 + b^2 - c^2 - d^2,           2*(b*c - a*d),         2*(b*d + a*c)},
          {          2*(b*c + a*d),   a^2 - b^2 + c^2 - d^2,         2*(c*d - a*b)},
          {          2*(b*d - a*c),           2*(c*d + a*b), a^2 - b^2 - c^2 + d^2}};
end quatToDcm;

function quatDeriv
  input Real q[4];
  input Real w[3];
  output Real qDot[4];
protected
  Real a,b,c,d;
algorithm
  a := q[1];
  b := q[2];
  c := q[3];
  d := q[4];
  qDot := 0.5 * {{ a,-b,-c,-d},
                 { b, a,-d, c},
                 { c, d, a,-b},
                 { d,-c, b, a}} * {0, w[1], w[2], w[3]};
end quatDeriv;

function eulerToQuat
  input Real euler[3] "phi, theta, psi";
  output Real q[4];
protected
  Real phi, theta, psi;
algorithm
  phi := euler[1];
  theta := euler[2];
  psi := euler[3];
  q[1] := cos(phi/2)*cos(theta/2)*cos(psi/2) +
          sin(phi/2)*sin(theta/2)*sin(psi/2);

  q[2] := sin(phi/2)*cos(theta/2)*cos(psi/2) -
          cos(phi/2)*sin(theta/2)*sin(psi/2);

  q[3] := cos(phi/2)*sin(theta/2)*cos(psi/2) +
          sin(phi/2)*cos(theta/2)*sin(psi/2);

  q[4] := cos(phi/2)*cos(theta/2)*sin(psi/2) -
          sin(phi/2)*sin(theta/2)*cos(psi/2);
end eulerToQuat;

function dcmToEuler
  input Real C_nb[3,3];
  output Real euler[3];
algorithm
  euler := {atan2(C_nb[3,2],C_nb[3,3]),
            asin(-C_nb[3,1]),
            atan2(C_nb[2,1],C_nb[1,1])};
end dcmToEuler;

function quatToEuler
  input Real q_nb[4];
  output Real euler[3];
algorithm
  euler := dcmToEuler(quatToDcm(q));
end quatToEuler;

block InertialAttitudeQuaternionBased "a quaternion based attitude computer"
  extends Modelica.Blocks.Interfaces.DiscreteBlock(startTime=0,samplePeriod=1.0/10);
  input Real w_ib[3];
  parameter Real euler_start[3] = {0,0,0};
  output Real C_nb[3,3];
  /*// TODO, gimbal lock, titterton pg. 47*/
  output Real q[4](start=eulerToQuat(euler_start));
  output Real euler[3];
  output Real phi, theta, psi;
  output Real phi_deg, theta_deg, psi_deg;
  output Real qNorm;
  constant Real rad2Deg = 180/Modelica.Constants.pi;
equation
  euler = {phi,theta,psi};
  euler * rad2Deg = {phi_deg, theta_deg, psi_deg};
algorithm
  when sampleTrigger then
    // TODO, nav computer pitch not in agreement
    q := quatDeriv(q,w_ib)*samplePeriod + q;
    C_nb := quatToDcm(q);
    qNorm := sqrt(q*q);
    euler := dcmToEuler(C_nb);
    if abs(1- qNorm) > 1e-3 then
      reinit(q[1],q[1]/qNorm);
      reinit(q[2],q[2]/qNorm);
      reinit(q[3],q[3]/qNorm);
      reinit(q[4],q[4]/qNorm);
    end if;
  end when;
end InertialAttitudeQuaternionBased;

/*block InertialPosition "an inertial position computer"*/
  /*extends Modelica.Blocks.Interfaces.DiscreteBlock(startTime=0,samplePeriod=1.0/10);*/
  /*input Real a_b[3];*/
  /*input Real w_ib[3];*/
  /*input Real C_br[3,3];*/
/*equation*/
  /*a_b = der(v_b) + cross(w_ib,v_b);*/
  /*v_r = transpose(C_br)*v_b;*/
  /*v_r = der(r_r) + cross(w_ir,r_r);*/
  /*v_b = der(C_br*r_r) + cross(w_ib,r_r);*/
/*end InertialPosition;*/

record NavigatorState
  Real phi;
  Real theta;
  Real psi;
  Real vN;
  Real vE;
  Real vD;
  Real L;
  Real l;
  Real h;
end NavigatorState;

record NavigatorInput
  Real wX, wY, wZ;
  Real fX, fY, fZ;
end NavigatorInput;

block NavigatorLinearization "
Strapdown Inertial Navigation Technology, Titterton, pg. 344
"
NavigatorState x;
protected
  Real cL = cos(L);
  Real sL = sin(L);
  Real tL = tan(L);
algorithm
  Real F[9,9] = {{              0, -(W*sL + vE*tL/R,        vN/R,                0,                 1/R,             0,                                -W*sL, 0,               -vE/R^2},
                 { W*sL + vE*tL/R,                0, W*cL + vE/R,             -1/R,                   0,             0,                                    0, 0,                vN/R^2}, 
                 {          -vN/R,     -W*cL - vE/R,           0,                0,               -tL/R,             0,                  -W*cL - vE/(R*cL^2), 0,             vE*tL/R^2},
                 {              0,              -fD,          fE,             vD/R, -2*(W*sL + vE*tL/R),          vN/R,           -vE*(2*W*cL + vE/(R*cL^2)), 0, (vE^2*tL - vN*vD)/R^2}, 
                 {             fD,                0,         -fN, 2*W*sL + vE*tL/R,      (vN*tL + vD)/R, 2*W*cL + vE/R, 2*W*(vN*cL - vD*sL) + vN*vE/(R*cL^2), 0,  -vE*(vN*tL + vD)/R^2},
                 {            -fE,               fN,           0,          -2*vN/R,    -2*(W*cL + vE/R),             0,                            2*W*vE*sL, 0,     (vN^2 + vE^2)/R^2},
                 {              0,                0,           0,              1/R,                   0,             0,                                    0, 0,               -vN/R^2},
                 {              0,                0,           0,                0,            1/(R*cL),             0,                         vE*tL/(R*cL), 0,          -vE/(R^2*cL)}};
  // TODO not sure if this matrix syntax will work
  Real G[6,6] = {{     -C_nb, zeros(3,3)},
                 {zeros(3,3),       C_nb}};
end NavigatorLinearization

end Navigation;

// vim:ts=2:sw=2:expandtab
