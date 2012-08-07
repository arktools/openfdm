within OpenFDM.Parts;

model ForceMoment "A rigid body force and moment."
  import SI=Modelica.SIunits;
  SI.Force F_b[3];
  SI.Torque M_b[3];
  Interfaces.RigidConnector fA;
equation
  fA.F_b + F_b = zeros(3);
  fA.M_b + M_b = zeros(3);
end ForceMoment;

// vim:ts=2:sw=2:expandtab:
