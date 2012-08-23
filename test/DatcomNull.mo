within test;

model DatcomNull


  import C=Modelica.Constants;
  import OpenFDM.*;
  import OpenFDM.Aerodynamics.*;
  import OpenFDM.Aerodynamics.Datcom.empty1D;
  import OpenFDM.Aerodynamics.Datcom.empty2D;

  constant Datcom.Tables datcomTables(    
      CL_Basic = empty1D,
      dCL_Flap  = empty1D,
      dCL_Elevator  = empty1D,
      dCL_PitchRate  = empty1D,
      dCL_AlphaDot  = empty1D,

      CD_Basic  = empty1D,
      dCD_Flap  = empty1D,
      dCD_Elevator  = empty1D,

      dCY_Beta  = empty1D,
      dCY_RollRate  = empty1D,

      dCl_Aileron  = empty1D,
      dCl_Beta  = empty1D,
      dCl_RollRate  = empty1D,
      dCl_YawRate  = empty1D,

      Cm_Basic = empty1D,
      dCm_Flap  = empty1D,
      dCm_Elevator  = empty1D,
      dCm_PitchRate  = empty1D,
      dCm_AlphaDot  = empty1D,

      dCn_Aileron  = empty1D,
      dCn_Rudder  = empty1D,
      dCn_Beta  = empty1D,
      dCn_RollRate  = empty1D,
      dCn_YawRate  = empty1D);

  inner World.Earth world;

  // init aircraft in steady level flight
  // can change pitch and throttle only
  // to obtain zero flight path angle at desired vt
  Parts.RigidReferencePoint p(
    // true airspeed
    //vt(start=6,fixed=false),
    // flight path angle
    //gamma(start=0,fixed=true),
    v_r(start={20,0,0},fixed={false,true,true}),
    // position fixed
    r_r(start={0,0,-1000},fixed={true,true,true}),
    // can change pitch, roll and heading fixed
    euler(start={0,0,0},fixed={true,false,true}),
    // no angular velocity, or acceleration
    w_ib(start={0,0,0},fixed={true,true,true}),
    z_b(start={0,0,0},fixed={true,true,true}),
    // no translational acceleration
    // not possible in z direction since no lift
    a_b(start={0,0,0},fixed={true,true,false})
    );


  OpenFDM.Control.AutoPilotConst pilot(
    throttle(start=0.5, fixed=false),
    elevator_deg(start=0, fixed=false),
    rudder_deg(start=0, fixed=false),
    aileron_deg(start=0, fixed=false),
    flap_deg(start=0, fixed=true));

  Datcom.ForceMoment aero(
    tables=datcomTables,
    elevator_deg=pilot.elevator_deg,
    rudder_deg=pilot.rudder_deg,
    aileron_deg=pilot.aileron_deg,
    flap_deg=pilot.flap_deg,
    s=1, b=1, cBar=1);


  Propulsion.Thruster motor(throttle=pilot.throttle);

  Parts.RigidBody structure(m=1,I_b=identity(3));
  Parts.RigidLink_B321 t_aero_rp(r_a={0,0,0}, angles={0,0,0});
  Parts.RigidLink_B321 t_motor(r_a={0,0,0}, angles={0,0,0});

equation

  connect(p.fA,structure.fA);

  connect(p.fA,t_motor.fA);
  connect(t_motor.fB,motor.fA);

  connect(p.fA,t_aero_rp.fA);
  connect(t_aero_rp.fB,aero.fA);

end DatcomNull;

// vim:ts=2:sw=2:expandtab:
