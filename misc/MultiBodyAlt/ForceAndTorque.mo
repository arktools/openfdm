within OpenFDM;

package ForceAndTorque

  import SI=Modelica.SIunits;

  model Base
    Interfaces.Frame frame;
    SI.Force force[3];
    SI.Torque torque[3];
  equation
    frame.f_0 = frame.C_0f*force;
    frame.t_0 = frame.C_0f*torque + 
      cross(frame.r_0,frame.f_0);
  end Base;

end ForceAndTorque;

// vim:ts=2:sw=2:expandtab:
