within Test;

model Environment "environment for multibody frame"
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  import Modelica.Mechanics.MultiBody.Frames.*;
  MultiBody.Interfaces.Frame frame;
  SI.Density rho "air density";  
  SI.Position asl "altitude above sea level";
  SI.Position agl "altitude above ground level";
  SI.Position groundAsl "altitude of ground above sea level";
  SI.Velocity wind_ECEF[3] "wind vector";
equation
  asl = -frame.r_0[3]; // TODO: should subtract radius of earth
  agl = asl - groundAsl; 

  // TODO
  rho = 1.225;
  wind_ECEF = {0,0,0};
  groundAsl = 0;

  // envronment exerts no force torques directly
  // only provides state dependent info for frame
  frame.f = {0,0,0};
  frame.t = {0,0,0};
end Environment;

