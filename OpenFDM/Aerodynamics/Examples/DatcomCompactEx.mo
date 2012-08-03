within OpenFDM.Aerodynamics.Examples;

model DatcomCompactEx

  import MB=MultiBodyOmc;
  import OpenFDM.Aerodynamics.Datcom;
  import OpenFDM.Aerodynamics.Datcom.empty2D;

  constant Datcom.TablesCompact tables(    
      AlphaTable = empty2D);

  model Body
    Airframe airframe(
      r_0(start={0,0,-10000}),
      v_0(start={10,0,0}));
    Datcom.ForceAndTorqueCompact aerodynamics(
      tables=tables,
      rudder_deg = 0,
      flap_deg = 0,
      elevator_deg = 0,
      aileron_deg = 0,
      coefs(s=1, b=1, cBar=1));
  equation
    connect(airframe.frame_a,aerodynamics.frame_b);
  end Body;

  inner MB.World world(n={0,0,1});
  Body body;

end DatcomCompactEx;

// vim:ts=2:sw=2:expandtab:
