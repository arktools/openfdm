within OpenFDM.Propulsion;

model Engine
  outer World world;
  Modelica.Mechanics.MultiBody.Parts.Body body;
  Interfaces.Frame frame_b;
  Real T[3];
  equation
    frame_b.f = T;
    frame_b.t = {0,0,0};
end Engine;

