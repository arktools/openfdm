within test;

model DatcomSimple

  import MB=MultiBodyOmc;
  import Aero=OpenFDM.Aerodynamics;
  import OpenFDM.Aerodynamics.Datcom;

  model Body

    OpenFDM.AirframeInitGlide airframe;

    Datcom.ForceAndTorqueSimple aerodynamics( 
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
      CYb = 0.01,
      // roll moment
      Clp = -0.1,
      Clda = 0.1,
      // pitch moment
      Cmq = -0.1,
      Cma = -0.1,
      Cmde = 0.1,
      // yaw moment
      Cnb = 0.1,
      Cnr = -0.1,
      Cndr = 0.1,
      // stall
      alphaStall_deg = 20,
      coefs(s=1, b=1, cBar=1)
    );
  equation
    connect(airframe.frame_a,aerodynamics.frame_b);
  end Body;

  inner OpenFDM.WorldNED world;
  Body body;

end DatcomSimple;

// vim:ts=2:sw=2:expandtab:
