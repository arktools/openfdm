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
    //vt(start=20,fixed=true),
    // flight path angle
    //gamma(start=0,fixed=true),
    v_r(start={20,0,0},fixed={true,true,true}),
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


  model AerodynamicsSimple
    input Real elevator_deg;
    input Real rudder_deg;
    input Real aileron_deg;
    input Real flap_deg;
    extends Aerodynamics.ForceMomentStabilityFrame(
      s=1,cBar=0.1,b=1);
  equation
    CL =
      (1.5/20)*alpha_deg + 
      0.00001*flap_deg +
      0.0001*elevator_deg +
      0.0001*q*cBar/(2*vt) +
      0.0001*alphaDot*alphaDot*cBar/(2*vt) +
      0;
    CD =
      0.001*CL^2 + 0.001 +
      0.00001*flap_deg +
      0.00001*elevator_deg + 
      0;
    CY =
      0.01*beta_deg +
      0.0001*p*b/(2*vt) +
      0; 
    Cl =
      0.00001*aileron_deg +
      (-0.1)*p*b/(2*vt) +
      0.0001*r*b/(2*vt) +
      0;
    Cm =
      (-0.00001)*(alpha_deg-5) +
      0.00001*flap_deg +
      (-0.1)*q +
      0.0001*elevator_deg +
      0;
    Cn =
      0.00001*aileron_deg +
      0.00001*p*b/(2*vt) +
      (-0.1)*r*b/(2*vt) +
      0;
  end AerodynamicsSimple;
  

  OpenFDM.Control.AutoPilotConst pilot;

  AerodynamicsSimple aero(
    elevator_deg=pilot.elevator_deg,
    rudder_deg=pilot.rudder_deg,
    aileron_deg=pilot.aileron_deg,
    flap_deg=pilot.flap_deg);

  OpenFDM.Propulsion.Thruster thrust(throttle=pilot.throttle);

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
