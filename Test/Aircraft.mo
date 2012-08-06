within Test;

model Aircraft

  inner World world;
  extends RigidReferencePoint;

  model Aerodynamics
    extends ForceMoment;
  equation
    F_b = {0,0,0};
    M_b = {0,0,0};
  end Aerodynamics;

  model Thrust
    extends ForceMoment;
  equation
    F_b = {0,0,0};
    M_b = {0,0,0};
  end Thrust;

  Aerodynamics aero;
  Thrust thrust;
  RigidBody structure(m=1,I_b=identity(3));
  RigidLink_B321 t_aero_rp(r_a={0,0,0}, angles={0,0,0});
  RigidLink_B321 t_motor(r_a={0,0,0}, angles={0,0,0});
equation
  connect(fA,structure.fA);

  connect(fA,t_motor.fA);
  connect(t_motor.fB,thrust.fA);

  connect(fA,t_aero_rp.fA);
  connect(t_aero_rp.fB,aero.fA);

end Aircraft;

// vim:ts=2:sw=2:expandtab:
