within Test;

model Rocket
  inner World world;
  extends RigidReferencePoint(
    euler(start={0,1,0},fixed={true,false,true}),
    r_r(start={0,0,-1},fixed=true));
  RigidBody structure(I_b=identity(3),m=0.1);
  SolidRocketMotor motor(
    mInert=0.1,
    mFuel=0.2,
    Ve=1000,
    mDot=1.0);
  RigidLink_B321 t(angles={0,0,0},r_a={0,0,0});
  SI.Position agl;
equation
  agl = world.agl(r_r);
  assert(r_r[3] <= 0, "hit ground");
  connect(fA,structure.fA);
  connect(structure.fA,t.fA);
  connect(t.fB,motor.fA);
end Rocket;

// vim:ts=2:sw=2:expandtab:
