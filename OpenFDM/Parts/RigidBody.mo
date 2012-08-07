within OpenFDM.Parts;

model RigidBody "A body with rotational and translational dynamics. The body is assumed to be rigid."
  import SI=Modelica.SIunits;
  extends Translational;
  SI.MomentOfInertia I_b[3,3] "inertial about the cm in the body frame";
protected
  SI.AngularMomentum H_b[3] "angular momentum";
equation
  H_b = I_b*fA.w_ib;
  fA.M_b = der(H_b) + cross(fA.w_ib,H_b);
end RigidBody;

// vim:ts=2:sw=2:expandtab:
