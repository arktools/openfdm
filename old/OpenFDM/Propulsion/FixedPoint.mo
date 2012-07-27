within OpenFDM.Propulsion;

model FixedPoint "Fixed Pt for testing purposes"
import Modelica.Mechanics.MultiBody.Frames;  
  Interfaces.Frame frame_fixed;

equation
  frame_fixed.r_0 = {0,0,0};
  frame_fixed.R = Frames.nullRotation();
  frame_fixed.f = {0,0,0};
  frame_fixed.t = {0,0,0};

end FixedPoint;