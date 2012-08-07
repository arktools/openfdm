within OpenFDM.Parts;

model PointMass "A point mass with translational, but no rotational dynamics."
  extends Translational;
equation
  fA.M_b = zeros(3);
end PointMass;

// vim:ts=2:sw=2:expandtab:
