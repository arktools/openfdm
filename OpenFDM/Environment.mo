within OpenFDM;

model Environment "environment for multibody frame"
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import MultiBodyOmc;
  import MultiBodyOmc.Frames.*;
  MultiBodyOmc.Interfaces.Frame frame;
  SI.Density rho "air density";  
  SI.Position asl "altitude above sea level";
  SI.Position agl "altitude above ground level";
  SI.Position groundAsl "altitude of ground above sea level";
  SI.Velocity wind_NED[3] "wind vector";
equation

  assert(agl > 0, "altitude below ground level");
  asl = -frame.r_0[3]; // TODO: should subtract radius of earth
  agl = asl - groundAsl; 

  // TODO
  rho = 1.225;
  wind_NED = {0,0,0};
  groundAsl = 0;

  // envronment exerts no force torques directly
  // only provides state dependent info for frame
  frame.f = {0,0,0};
  frame.t = {0,0,0};
end Environment;

// vim:ts=2:sw=2:expandtab:
