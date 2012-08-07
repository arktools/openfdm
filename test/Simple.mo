within test;

model Simple
  import OpenFDM.*;
  inner World world;
  RigidReferencePoint p(
    r_r(start={0,0,-10},fixed=true),
    euler(start={0,1,0},fixed=true));
  ForceMoment fM1(
    F_b=0.0001*{1,1,1},
    M_b=0.0001*{1,1,1});
  ForceMoment fM2(
    F_b=0.0001*{1,1,1},
    M_b=0.0001*{1,1,1});
  RigidBody b1(m=1,I_b=identity(3));
  PointMass b2(m=1);
  RigidLink_B321 t(r_a={1,2,3}, angles={1,1,1});
equation
  connect(p.fA,b1.fA);
  connect(fM1.fA,b1.fA);
  connect(fM2.fA,b2.fA);
  connect(b1.fA,t.fA);
  connect(t.fB,b2.fA);
end Simple;

// vim:ts=2:sw=2:expandtab:
