within OpenFDM.Aerodynamics.Examples;

model ForceAndTorqueEx

  import Aero=OpenFDM.Aerodynamics;
  import MB=Modelica.Mechanics.MultiBody;

  Aero.BodyFrame.ForceAndTorque aerodynamics;
  MB.Parts.Bdoy body;

equation

  connect(body.frame_a,aerodynamics.frame_b);

end ForceAndTorqueEx;

// vim:ts=2:sw=2:expandtab:
