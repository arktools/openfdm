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

function localGravity
  // see tittterton pg. 57 for local gravity model
  input NavigatorState x;
  output Real g_l_n[3]; 
algorithm
  g_l_n := {0,0,9.8};
end localGravity;

record NavigatorState
  Real phi, theta, psi, vN, vE, vD, L, l, h;
end NavigatorState;

record NavigatorInput
  Real wX, wY, wZ;
  Real fX, fY, fZ;
end NavigatorInput;

function NavigatorF "
Strapdown Inertial Navigation Technology, Titterton, pg. 344
"
  input NavigatorState x(vN=vN, vE=vE, vD=vD);
  input Real f_n[3];
  input Real W;
  input Real R0;

  output Real F[9,9];

protected
  Real vN, vE, vD;
  Real fN, fE, fD;
  Real cL, sL, tL;
  Real R;

algorithm
  cL := cos(x.L);
  sL := sin(x.L);
  tL := tan(x.L);
  fN := f_n[1];
  fE := f_n[2];
  fD := f_n[3];
  R := x.h + R0;
  F := {
  {              0, -(W*sL + vE*tL/R),        vN/R,                0,                 1/R,             0,                                -W*sL, 0,               -vE/R^2},
  { W*sL + vE*tL/R,                 0, W*cL + vE/R,             -1/R,                   0,             0,                                    0, 0,                vN/R^2}, 
  {          -vN/R,      -W*cL - vE/R,           0,                0,               -tL/R,             0,                  -W*cL - vE/(R*cL^2), 0,             vE*tL/R^2},
  {              0,               -fD,          fE,             vD/R, -2*(W*sL + vE*tL/R),          vN/R,           -vE*(2*W*cL + vE/(R*cL^2)), 0, (vE^2*tL - vN*vD)/R^2}, 
  {             fD,                 0,         -fN, 2*W*sL + vE*tL/R,      (vN*tL + vD)/R, 2*W*cL + vE/R, 2*W*(vN*cL - vD*sL) + vN*vE/(R*cL^2), 0,  -vE*(vN*tL + vD)/R^2},
  {            -fE,                fN,           0,          -2*vN/R,    -2*(W*cL + vE/R),             0,                            2*W*vE*sL, 0,     (vN^2 + vE^2)/R^2},
  {              0,                 0,           0,              1/R,                   0,             0,                                    0, 0,               -vN/R^2},
  {              0,                 0,           0,                0,            1/(R*cL),             0,                         vE*tL/(R*cL), 0,          -vE/(R^2*cL)},
  {              0,                 0,           0,                0,                   0,            -1,                                    0, 0,                     0}};
end NavigatorF;

function NavigatorG "
Strapdown Inertial Navigation Technology, Titterton, pg. 344
"
  input Real C_nb[3,3];
  output Real G[6,6];
algorithm
  G := {
  {     -C_nb, zeros(3,3)},
  {zeros(3,3),       C_nb}};
end NavigatorG;

block InertialNavigationSystem "a quaternion based INS" 
  extends Modelica.Blocks.Interfaces.DiscreteBlock(startTime=0,samplePeriod=1.0/1000);
  input NavigatorInput u;

  parameter Real W = 0;   
  parameter Real R0 = 6.3891e6;
  parameter Real euler_start[3] = {0,0,0};

  output NavigatorState x(phi=phi,theta=theta,psi=psi,vN=vN,vE=vE,vD=vD,L=L,l=l,h=h);
  output Real C_nb[3,3];
  // TODO, gimbal lock, titterton pg. 47
  output Real euler[3];
  output Real phi_deg, theta_deg, psi_deg;
  constant Real rad2Deg = 180/Modelica.Constants.pi;
  output Real q[4](start=eulerToQuat(euler_start));
  output Real qNorm;
  output Real qDot[4];
  /*[>output Real F[9,9], G[6,6];<]*/
protected
  Real phi, theta, psi, vN, vE, vD, L, l, h;
  Real sL, cL, R, LDot, lDot, hDot, vNDot, vEDot, vDDot;
  Real f_n[3];
  Real g_l_n[3];
equation
  euler = {phi,theta,psi};
  euler * rad2Deg = {phi_deg, theta_deg, psi_deg};
algorithm
  when sampleTrigger then
    // TODO, nav computer pitch not in agreement
    qDot := quatDeriv(q,{u.wX, u.wY, u.wZ});
    q := q + qDot*samplePeriod;
    qNorm := sqrt(q*q);
    if abs(1- qNorm) > 1e-3 then
      q := q/qNorm;
    end if;
    C_nb := quatToDcm(q);
    euler := dcmToEuler(C_nb);
    sL := sin(L);
    cL := cos(L);
    R := R0 + h;
    LDot := vN/R;
    lDot := vE/(R*cL);
    hDot := -vD;
    f_n := C_nb * {u.fX, u.fY, u.fZ};
    g_l_n := localGravity(x);
    vNDot := f_n[1] - vE*(2*W + lDot)*sL + vD*LDot + g_l_n[1];
    vEDot := f_n[2] + vN*(2*W + lDot)*sL + vD*(2*W+lDot)*cL + g_l_n[2];
    vDDot := f_n[3] - vE*(2*W + lDot)*cL - vN*LDot + g_l_n[3];
    vN := vN + vNDot*samplePeriod;
    vE := vE + vEDot*samplePeriod;
    vD := vD + vDDot*samplePeriod;
    L := L + LDot*samplePeriod;
    l := l + lDot*samplePeriod;
    h := h + hDot*samplePeriod;
    /*[>F := NavigatorF(x, f_n, W, R0);<]*/
    /*[>G := NavigatorG(C_nb);<]*/
  end when;
end InertialNavigationSystem;

end Navigation;

// vim:ts=2:sw=2:expandtab
