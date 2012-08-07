within test;

model Rocket
  import OpenFDM.*;
  inner World world;
  Parts.RigidReferencePoint p(
    r_r(start={0,0,-1},fixed=true),
    euler(start={0,1,0},fixed=true));
  Parts.RigidBody structure(I_b=identity(3),m=0.1);
  Propulsion.SolidRocketMotor motor(
    mInert=0.1,
    mFuel=0.2,
    Ve=1000,
    mDot=1.0);
  Parts.RigidLink_B321 t(angles={0,0,0},r_a={0,0,0});
equation
  connect(p.fA,structure.fA);
  connect(structure.fA,t.fA);
  connect(t.fB,motor.fA);
end Rocket;

// vim:ts=2:sw=2:expandtab:
