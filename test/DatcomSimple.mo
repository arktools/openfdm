within test;

model DatcomSimple


  import C=Modelica.Constants;
  import OpenFDM.*;
  import OpenFDM.Aerodynamics.*;
  import OpenFDM.Aerodynamics.Datcom.empty1D;
  import OpenFDM.Aerodynamics.Datcom.empty2D;

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
    a_b(start={0,0,0},fixed={true,true,false})
    );

  model Thrust
    extends Parts.ForceMoment;
    input Real throttle(start=0.3,min=0,max=1,fixed=true);
  equation
    der(throttle) = 0;
    F_b = throttle*{0.1,0,0};
    M_b = {0,0,0};
  end Thrust;

  Datcom.ForceMomentSimple aero( 
    // controls
    aileron_deg = 0,
    rudder_deg = 0,
    elevator_deg = 0,
    flap_deg = 0,
    // lift
    CL0 = 0.1,
    CLa = 0.1/20.0,
    // drag
    CD0 = 0.01,
    CDCL = 0.01,
    // side force
    CYb = 0, //0.01,
    // roll moment
    Clp = -0.1,
    Clda = 0.1,
    // pitch moment
    Cmq = -0.1,
    Cma = -0.1,
    Cmde = 0.1,
    // yaw moment
    Cnb = 0, //0.1,
    Cnr = -0.1,
    Cndr = 0.1,
    // stall
    alphaStall_deg = 20,
    s=1, b=1, cBar=1);

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

end DatcomSimple;

// vim:ts=2:sw=2:expandtab:
