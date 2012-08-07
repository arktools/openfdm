within Test;

model Rocket
  inner World world;
  extends RigidReferencePoint(
    euler(start={0,1,0},fixed={true,false,true}),
    r_r(start={0,0,-1},fixed=true));
  RigidBody structure(I_b=identity(3),m=0.1);
  model RocketMotor
    RigidConnector fA;
    model Thrust
      extends ForceMoment;
      SI.MassFlowRate mDot;
      SI.Velocity Ve = 1000;
    equation
      F_b = {0,0,-mDot*Ve};
      M_b = {0,0,0}; 
    end Thrust;
    model Structure
      extends RigidBody;
      SI.Mass mFuel(start=0.1,fixed=true);
      SI.Mass mInert=0.1;
      SI.MassFlowRate mDot(start=0.1);
    equation
      der(mDot) = 0;
      der(mFuel) = -mDot;
      when (mFuel <= 0) then
        reinit(mFuel,0);
        reinit(mDot,0);
      end when;
      I_b = m*identity(3);
      m = mInert + mFuel;
    end Structure;
    Thrust thrust;
    Structure structure;
  equation
    structure.mDot = thrust.mDot;
    connect(structure.fA,fA);
    connect(thrust.fA,structure.fA);
  end RocketMotor;
  RocketMotor motor;
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
