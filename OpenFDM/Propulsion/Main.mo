within OpenFDM.Propulsion;

model Main
  
  Engine engine;
  EnginePosition engPos;
  FixedPoint fixed;

equation
  connect(engine.frame_b, engPos.frame_b);
  connect(fixed.frame_fixed, engPos.frame_a);
end Main;