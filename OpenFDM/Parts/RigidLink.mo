within OpenFDM.Parts;

partial model RigidLink "Requires C_ba definition to be complete."
  import SI=Modelica.SIunits;
  input SI.Position r_a[3] "position vector from fA to fB, resolved in fA";
  input SI.Angle angles[3] "rotation angles from fA into fB";
  Interfaces.RigidConnector fA, fB;
  Real C_ba[3,3] "rotation matrix from fA into fB";
equation
  fA.r_r + transpose(fA.C_br)*r_a = fB.r_r;
  C_ba*fA.v_b = fB.v_b;
  C_ba*fA.a_b = fB.a_b;
  zeros(3) = C_ba*fA.F_b + fB.F_b;
  C_ba*fA.C_br = fB.C_br;
  C_ba*fA.w_ib = fB.w_ib;
  C_ba*fA.z_b = fB.z_b;
  zeros(3) = C_ba*fA.M_b + 
    cross(C_ba*r_a,fB.F_b) +
    fB.M_b;
end RigidLink;

model RigidLink_B321 "A body 3-2-1 rotation sequence rigid connector"
  extends RigidLink;
equation
  C_ba = T1(angles[1])*T2(angles[2])*T3(angles[3]);
end RigidLink_B321;

// vim:ts=2:sw=2:expandtab:
