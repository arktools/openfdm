within OpenFDM.Parts;

partial model Translational "Translational kinematics of a rigid body. Requires moment definition to be complete. Used as a base for point mass and rigid body."
  outer World.WorldBase world;
  import SI=Modelica.SIunits;
  Interfaces.RigidConnector fA;
  SI.Mass m "mass";
protected
  SI.Momentum L_b[3] "linear momentum";
equation
  L_b = m*(fA.v_b + cross(fA.w_ib,fA.C_br*fA.r_r));
  fA.F_b + fA.C_br*world.g_r(fA.r_r) = 
    der(L_b) + cross(fA.w_ib,L_b);
end Translational;

// vim:ts=2:sw=2:expandtab:
