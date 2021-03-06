within OpenFDM.Aircraft;

package Aerosonde

model Aircraft

  import C=Modelica.Constants;
  import OpenFDM.*;
  import OpenFDM.Aerodynamics.Datcom.empty1D;
  import OpenFDM.Aerodynamics.Datcom.empty2D;
  import OpenFDM.Aircraft.Aerosonde;
  inner World.Earth world;

  Real ePhi_deg;
  Real eTheta_deg;
  Real ePsi_deg;

  // init aircraft in steady level flight
  // can change pitch and throttle only
  // to obtain zero flight path angle at desired vt
  Parts.RigidReferencePoint p(
    // true airspeed
    //vt(start=20,fixed=false),
    // flight path angle
    //gamma(start=0,fixed=true),
    v_r(start={40,0,0},fixed={true,true,true}),
    // position fixed
    r_r(start={0,0,-1000},fixed={true,true,true}),
    // can change pitch, roll and heading fixed
    euler(start={0,0,0},fixed={false,false,false}),
    // no angular velocity, or acceleration
    eulerDot(start={0,0,0},fixed={true,true,true}),
    z_b(start={0,0,0},fixed={true,true,true}),
    // no translational acceleration
    a_b(start={0,0,0},fixed={true,true,true}));


  OpenFDM.Control.AutoPilotConst pilot;

  OpenFDM.Navigation.InertialNavigationSystem nav(
    u.wX = p.w_ib[1],
    u.wY = p.w_ib[2],
    u.wZ = p.w_ib[3],
    u.fX = p.a_b[1],
    u.fY = p.a_b[2],
    u.fZ = p.a_b[3],
    euler_start={0,-0.046,0});

  Aerodynamics.Datcom.ForceMoment aero(
    tables=Aerosonde.Datcom.tables,
    rudder_deg = pilot.rudder_deg,
    flap_deg = pilot.flap_deg,
    elevator_deg = pilot.elevator_deg,
    aileron_deg = pilot.aileron_deg,
    s=1.04322,
    b=3.377,
    cBar=0.4602);

  OpenFDM.Propulsion.Thruster thruster(
    maxThrust=100,
    throttle=pilot.throttle);

  Parts.RigidBody structure(
    m=22.18,
    I_b={{101.686,0,0},{0,43.071,0},{0,0,85.463}});

  Parts.RigidLink_B321 t_aero_rp(
    r_a={-0.05,0,0},
    angles={0,0,0});

  Parts.RigidLink_B321 t_motor(
    r_a={-0.5639,0,-0.09144},
    angles={0,0,0});

equation

  assert(p.w_ib[1] < 1, "rolling too fast");
  assert(p.w_ib[2] < 1, "pitching too fast");
  assert(p.w_ib[3] < 1, "yawing too fast");
  assert(p.z_b[1] < 100, "Zx too large");
  assert(p.z_b[2] < 100, "Zy too large");
  assert(p.z_b[3] < 100, "Zz too large");
  assert(p.v_b[1] < 100, "Vx too fast");
  assert(p.v_b[2] < 100, "Vy too fast");
  assert(p.v_b[3] < 100, "Vz too fast");
  assert(p.a_b[1] < 100, "Ax too large");
  assert(p.a_b[2] < 100, "Ay too large");
  assert(p.a_b[3] < 100, "Az too large");

  connect(p.fA,structure.fA);

  connect(p.fA,t_motor.fA);
  connect(t_motor.fB,thruster.fA);

  connect(p.fA,t_aero_rp.fA);
  connect(t_aero_rp.fB,aero.fA);

  ePhi_deg = nav.phi_deg - p.phi_deg;
  eTheta_deg = nav.theta_deg - p.theta_deg;
  ePsi_deg = nav.psi_deg - p.psi_deg;

end Aircraft;

end Aerosonde;

// vim:ts=2:sw=2:expandtab:
