within test;

model Aircraft

  import OpenFDM.*;

  inner World.Earth world;

  Parts.RigidReferencePoint p(
    r_r(start={0,0,-1000},fixed=true),
    euler(start={0,0,0},fixed=true));

  model Thrust
    extends Parts.ForceMoment;
  equation
    F_b = {0,0,0};
    M_b = {0,0,0};
  end Thrust;

  model SimpleAerodynamics
    extends Aerodynamics.AerodynamicForceMoment(
      vt(start=10,fixed=true)
    );
  equation
    F_b = {0,0,0};
    M_b = {0,0,0};
  end SimpleAerodynamics;
  
  SimpleAerodynamics aero;

  Thrust thrust;
  Parts.RigidBody structure(m=1,I_b=identity(3));
  Parts.RigidLink_B321 t_aero_rp(r_a={0,0,0}, angles={0,0,0});
  Parts.RigidLink_B321 t_motor(r_a={0,0,0}, angles={0,0,0});

equation

  connect(p.fA,structure.fA);

  connect(p.fA,t_motor.fA);
  connect(t_motor.fB,thrust.fA);

  connect(p.fA,t_aero_rp.fA);
  connect(t_aero_rp.fB,aero.fA);

end Aircraft;

// vim:ts=2:sw=2:expandtab:
