within test;

model Aircraft

  import OpenFDM.*;
  import C=Modelica.Constants;

  inner World.Earth world;

  Parts.RigidReferencePoint p(
    r_r(start={0,0,-1000},fixed={true,true,true}),
    euler(start={0,0,0},fixed={true,false,true}),
    w_ib(start={0,0,0},fixed={true,true,true}),
    z_b(start={0,0,0},fixed={true,true,true}),
    v_b(start={10,0,0},fixed={false,true,false}),
    a_b(start={0,0,0},fixed={true,true,true})
    );

  model Thrust
    extends Parts.ForceMoment;
  equation
    F_b = {0,0,0};
    M_b = {0,0,0};
  end Thrust;

  model AerodynamicsSimple
    extends Aerodynamics.StabilityFrame.ForceMoment(
      s=1,cBar=1,b=1);
  equation
    CL = (1.5/20)*alpha_deg;
    CD = 0.001*CL^2 + 0.001;
    CY = 0;
    Cl = -0.1*p;
    Cm = -0.1*q - 0.00001*(alpha_deg-5);
    Cn = -0.1*r;
  end AerodynamicsSimple;
  
  AerodynamicsSimple aero;

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
