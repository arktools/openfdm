within test;

model AerosondeModel

  import C=Modelica.Constants;
  import OpenFDM.*;
  import OpenFDM.Aerodynamics.*;
  import OpenFDM.Aerodynamics.Datcom.empty1D;
  import OpenFDM.Aerodynamics.Datcom.empty2D;
  import OpenFDM.Aircraft.Aerosonde;
  inner World.Earth world;

  // init aircraft in steady level flight
  // can change pitch and throttle only
  // to obtain zero flight path angle at desired vt
  Parts.RigidReferencePoint p(
    // true airspeed
    //vt(start=6,fixed=false),
    // flight path angle
    //gamma(start=0,fixed=true),
    v_r(start={20,0,0},fixed={true,true,true}),
    // position fixed
    r_r(start={0,0,-1000},fixed={true,true,true}),
    // can change pitch, roll and heading fixed
    euler(start={0,0,0},fixed={true,false,false}),
    // no angular velocity, or acceleration
    w_ib(start={0,0,0},fixed={true,true,true}),
    z_b(start={0,0,0},fixed={true,true,true}),
    // no translational acceleration
    a_b(start={0,0,0},fixed={true,true,true}));


  model Thrust
    extends Parts.ForceMoment;
    Modelica.Blocks.Nonlinear.Limiter sat(
      uMax=1,
      uMin=0);
    input Real throttle;
  equation
    sat.u = throttle;
    F_b = sat.y*{10,0,0};
    M_b = {0,0,0};
  end Thrust;

  model Pilot "a constant input autopilot"
    Real throttle(start=0.3, fixed=false);
    Real elevator_deg(start=0, fixed=false);
    Real rudder_deg(start=0, fixed=true);
    Real aileron_deg(start=0, fixed=true);
    Real flap_deg(start=0, fixed=true);
  equation
    der(throttle) = 0;
    der(elevator_deg) = 0;
    der(rudder_deg) = 0;
    der(aileron_deg) = 0;
    der(flap_deg) = 0;
  end Pilot;

  Pilot pilot;

  Datcom.ForceMoment aero(
    tables=Aerosonde.Datcom.tables,
    rudder_deg = pilot.rudder_deg,
    flap_deg = pilot.flap_deg,
    elevator_deg = pilot.elevator_deg,
    aileron_deg = pilot.aileron_deg,
    s=0.1, b=1, cBar=0.1);

  Thrust thrust(throttle=pilot.throttle);
  Parts.RigidBody structure(m=1,I_b=1*identity(3));
  Parts.RigidLink_B321 t_aero_rp(r_a={0,0,0}, angles={0,0,0});
  Parts.RigidLink_B321 t_motor(r_a={0,0,0}, angles={0,0,0});

equation

  assert(p.w_ib[1] < 1, "rolling too fast");
  assert(p.w_ib[2] < 1, "pitching too fast");
  assert(p.w_ib[3] < 1, "yawing too fast");
  assert(p.v_b[1] < 100, "Vx too fast");
  assert(p.v_b[2] < 100, "Vy too fast");
  assert(p.v_b[3] < 100, "Vx too fast");

  connect(p.fA,structure.fA);

  connect(p.fA,t_motor.fA);
  connect(t_motor.fB,thrust.fA);

  connect(p.fA,t_aero_rp.fA);
  connect(t_aero_rp.fB,aero.fA);

end AerosondeModel;

// vim:ts=2:sw=2:expandtab:
