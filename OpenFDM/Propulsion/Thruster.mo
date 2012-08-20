within OpenFDM.Propulsion;

model Thruster
  extends Parts.ForceMoment;
  parameter Real maxThrust = 10;
  Modelica.Blocks.Nonlinear.Limiter sat(
    uMax=1,
    uMin=0);
  input Real throttle;
equation
  sat.u = throttle;
  F_b = sat.y*{maxThrust,0,0};
  M_b = {0,0,0};
end Thruster;
// vim:ts=2:sw=2:expandtab
