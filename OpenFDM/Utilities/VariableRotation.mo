within OpenFDM.Utilities;

model VariableRotation
  "Variable rotation of frame_b with respect to frame_a"
  import MultiBodyOmc.Frames.*;
  import MultiBodyOmc.Types;
  import MultiBodyOmc.*;
  import SI = Modelica.SIunits;
  Interfaces.Frame_a frame_a
    "Coordinate system fixed to the component with one cut-force and cut-torque";
  Interfaces.Frame_b frame_b
    "Coordinate system fixed to the component with one cut-force and cut-torque";
  parameter Integer axis(min=1,max=3) 
    " Axis of rotation in frame_a (= same as in frame_b)";
  SI.Angle angle
    " Angle to rotate frame_a around axis n into frame_b";
  SI.AngularVelocity angleDot "derivative of angle";
  Frames.Orientation R_rel "relative rotation from frame_a to frame_b";
equation
  //Connections.branch(frame_a.R, frame_b.R);
  assert(cardinality(frame_a) > 0 or cardinality(frame_b) > 0,
    "Neither connector frame_a nor frame_b of FixedRotation object is connected");

  R_rel = axisRotation(2,angle,angleDot);

  /* Relationships between quantities of frame_a and frame_b */
  frame_b.r_0 = frame_a.r_0;
  frame_b.R = Frames.absoluteRotation(frame_a.R, R_rel);
  zeros(3) = frame_a.f + Frames.resolve1(R_rel, frame_b.f);
  zeros(3) = frame_a.t + Frames.resolve1(R_rel, frame_b.t);
end VariableRotation;

// vim:ts=2:sw=2:expandtab
