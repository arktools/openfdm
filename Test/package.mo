package Test

import C=Modelica.Constants;

function C_rb
  input Real angle[3];
  output Real T[3,3] = identity(3);
annotation(Inline=true);
end C_rb;

function C_br
  input Real angle[3];
  output Real T[3,3] = identity(3);
annotation(Inline=true);
end C_br;

model World

  function g
    input Real r_ecef[3];
    output Real g[3];
  algorithm
    g := {0,0,9.8};
  annotation(Inline=true);
  end g;

  function agl
    input Real r_ecef[3];
    output Real agl;
  algorithm
    agl := -r_ecef[3];
  annotation(Inline=true);
  end agl;

end World;

expandable connector RigidConnector
  Real r_r[3];
  Real v_b[3];
  Real a_b[3];
  flow Real F_b[3];
  Real C_br[3,3];
  Real w_ib[3];
  Real z_b[3]; 
  flow Real M_b[3];
end RigidConnector;

model RigidLink
  input Real r_a[3]={1,1,1};
  input Real angles[3]={1,1,1};
  input Integer sequence[3] = {3,2,1};
  RigidConnector fA, fB;
protected
  Real C_ba[3,3] = identity(3); // TODO
equation
  fA.r_r + transpose(fA.C_br)*r_a = fB.r_r;
  C_ba*fA.v_b = fB.v_b;
  C_ba*fA.a_b = fB.a_b;
  C_ba*fA.F_b + fB.F_b = zeros(3);
  C_ba*fA.C_br = fB.C_br;
  C_ba*fA.w_ib = fB.w_ib;
  C_ba*fA.z_b = fB.z_b;
  C_ba*fA.M_b + fB.M_b = zeros(3);
end RigidLink;

model ForceMoment
  Real F_b[3], M_b[3];
  RigidConnector fA;
equation
  fA.F_b + F_b = zeros(3);
  fA.M_b + M_b = zeros(3);
end ForceMoment;

partial model LinearDynamics
  outer World world;
  Real m;
  RigidConnector fA;
protected
  Real L_b[3];
equation
  L_b = m*(fA.v_b + cross(fA.w_ib,fA.C_br*fA.r_r));
  fA.F_b + fA.C_br*world.g(fA.r_r) = 
    der(L_b) + cross(fA.w_ib,L_b);
end LinearDynamics;

model PointMass
  extends LinearDynamics;
equation
  fA.M_b = zeros(3);
end PointMass;

model RigidBody
  extends LinearDynamics;
  Real I_b[3,3];
  Real H_b[3];
equation
  H_b = I_b*fA.w_ib;
  fA.M_b = der(H_b) + cross(fA.w_ib,H_b);
end RigidBody;

model ReferencePoint
  RigidConnector fA;
  Real w_ir[3] = {0,0,0} "ref frame ang rate wrt inertial";
  Real euler[3];
equation
  fA.v_b = fA.C_br*der(fA.r_r);
  fA.a_b = der(fA.v_b);
  fA.F_b = zeros(3);
  fA.w_ib = w_ir + der(euler); // TODO*/
  fA.C_br = identity(3); // TODO
  fA.z_b = der(fA.w_ib);
  fA.M_b = zeros(3);
  // angle wrap
  for i in 1:size(euler,1) loop
    when (euler[i] > C.pi) then
      reinit(euler[i],pre(euler[i])-2*C.pi);
    end when;
  end for;
end ReferencePoint;

model Simple
  ForceMoment fM1(F_b={1,1,1},M_b={1,1,1});
  ForceMoment fM2(F_b={1,1,1},M_b={1,1,1});
  RigidBody b1(m=1,I=identity(3));
  PointMass b2(m=1);
  RigidLink t(
    sequence={3,2,1},
    angles={0,0,0},
      r_a={1,2,3});
  ReferencePoint p; 
equation
  connect(fM1.fA,b1.fA);
  connect(fM2.fA,b2.fA);
  connect(b1.fA,t.fA);
  connect(b2.fA,t.fB);
  connect(p.fA,b1.fA);
end Simple;

model RocketThrust
  extends ForceMoment;
  Real mDot;
  Real Ve;
equation
  F_b = {0,0,-mDot*Ve};
  M_b = {0,0,0}; 
end RocketThrust;

model RocketMotor
  extends RigidBody;
  Real Ve = 1000;
  Real mFuel(start=1,fixed=true);
  Real mInert=0.1;
  Real mDot;
equation
  mDot = -der(mFuel);
  if (mFuel > 0) then
    der(mFuel) = -0.1;
  else
    der(mFuel) = 0;
  end if;
  I_b = m*identity(3);
  m = mInert + mFuel;
end RocketMotor;

model Structure
  extends RigidBody;
equation
  I_b = identity(3);
  m = 1;
end Structure;

model Rocket
  inner World world;
  ReferencePoint p;
  Structure structure;
  RocketMotor motor;
  RocketThrust thrust(
    Ve=motor.Ve,
    mDot=motor.mDot);
  RigidLink t(
    sequence={3,2,1},
    angles={0,0,0},
      r_a={1,2,3});
  Real agl;
equation
  agl = world.agl(p.fA.r_r);
  assert(p.fA.r_r[3] < 0, "hit ground");
  connect(p.fA,structure.fA);
  connect(t.fA,structure.fA);
  connect(motor.fA,t.fB);
  connect(motor.fA,thrust.fA);
end Rocket;

end Test;

// vim:ts=2:sw=2:expandtab:
