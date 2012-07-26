within OpenFDM.Propulsion;

model EnginePosition "Translation from Aircraft c.m. to Engine c.m., fixed in body-frame"
  Interfaces.Frame frame_a;
  Interfaces.Frame frame_b;
  Real r[3] = {1.0,0,0};

equation
  frame_b.r_0 = frame_a.r_0 + r;
  frame_b.R = frame_a.R;
  frame_b.f = frame_a.f;

end EnginePosition;