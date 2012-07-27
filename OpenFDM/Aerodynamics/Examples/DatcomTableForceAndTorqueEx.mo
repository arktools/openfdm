within OpenFDM.Aerodynamics.Examples;

model DatcomTableStabilityFrameAeroObject
  import Aero=OpenFDM.Aerodynamics;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));

  Aero.DatcomTables.DatcomForceAndTorqueTable aerodynamics(
      dFlap = 0,
      dElevator = 0,
      dAileron = 0,
      tables=Aero.DatcomTables.NullAircraft.datcomTables,
      // controls
      coefs(s=1, b=1, cBar=1));
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
end DatcomTableStabilityFrameAeroObject;

model DatcomTableForceAndTorqueEx
  import MB=Modelica.Mechanics.MultiBody;
  inner MB.World world(n={0,0,1});
  DatcomTableStabilityFrameAeroObject body_s;
end DatcomTableForceAndTorqueEx;


// vim:ts=2:sw=2:expandtab:
