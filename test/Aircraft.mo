within test;

model Aircraft

  import OpenFDM.*;
  import C=Modelica.Constants;

  inner World.Earth world;

  // init aircraft in steady level flight
  // can change pitch and throttle only
  // to obtain zero flight path angle at desired vt
  Parts.RigidReferencePoint p(
    // true airspeed
    //vt(start=6,fixed=false),
    // flight path angle
    //gamma(start=0,fixed=true),
    v_r(start={6,0,0},fixed={false,true,true}),
    // position fixed
    r_r(start={0,0,-1000},fixed={true,true,true}),
    // can change pitch, roll and heading fixed
    euler(start={0,0,0},fixed={true,false,true}),
    // no angular velocity, or acceleration
    w_ib(start={0,0,0},fixed={true,true,true}),
    z_b(start={0,0,0},fixed={true,true,true}),
    // no translational acceleration
    a_b(start={0,0,0},fixed={true,true,true})
    );

  model Thrust
    extends Parts.ForceMoment;
    input Real throttle(start=0.3,min=0,max=1,fixed=true);
  equation
    der(throttle) = 0;
    F_b = throttle*{0.1,0,0};
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
