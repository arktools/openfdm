within OpenFDM;

package Propulsion

  import SI=Modelica.SIunits;

  model ConstantThrust
    parameter SI.Force thrust = 100 "thrust force";
    extends ForceAndTorque.Base;
  equation
    force = {thrust,0,0};
    torque = {0,0,0};
  end ConstantThrust;

end Propulsion;

// vim:ts=2:sw=2:expandtab:
