within OpenFDM.Aerodynamics.Examples;

model DatcomSimpleEx

  import MB=MultiBodyOmc;
  import Aero=OpenFDM.Aerodynamics;
  import OpenFDM.Aerodynamics.Datcom;

  model Body

    Airframe airframe(
      r_0(start={0,0,-10000}),
      v_0(start={10,0,0}));

    Datcom.ForceAndTorqueSimple aerodynamics( 
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
      alpha_stall_deg = 20,
      coefs(s=1, b=1, cBar=1)
    );
  equation
    connect(airframe.frame_a,aerodynamics.frame_b);
  end Body;

  inner MB.World world(n={0,0,1});
  Body body;

end DatcomSimpleEx;

// vim:ts=2:sw=2:expandtab:
