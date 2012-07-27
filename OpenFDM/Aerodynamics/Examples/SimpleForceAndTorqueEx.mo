within OpenFDM.Aerodynamics.Examples;

model SimpleStabilityFrameAeroObject
  import Aero=OpenFDM.Aerodynamics;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));
  Aero.StabilityFrame.SimpleForceAndTorque aerodynamics(
    coefs(
      // lift
      CL0 = .1,
      CLa = .1/20.0,
      // drag
      CD0 = .01,
      CDCL = .01,
      // side force
      CYb = .01,
      // roll moment
      Clp = -.1,
      Clda = .1,
      Clda = .1,
      // pitch moment
      Cmq = -.1,
      Cma = -.1,
      Cmde = .1;
      // yaw moment
      Cnb = .1;
      Cnr = -.1;
      Cndr = .1;
      s=1, b=1, cBar=1));
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
end SimpleStabilityFrameAeroObject;

model SimpleForceAndTorqueEx
  import MB=Modelica.Mechanics.MultiBody;
  inner MB.World world(n={0,0,1});
  SimpleStabilityFrameAeroObject body_s;
end SimpleForceAndTorqueEx;

// vim:ts=2:sw=2:expandtab:
