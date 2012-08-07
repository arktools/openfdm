within OpenFDM.Aerodynamics; 

partial model ForceMoment "partial force model that computes aerodynamic relavent properties, still required to define F_b, M_B"
  extends Parts.ForceMoment;
  import SI=Modelica.SIunits;
  outer World.WorldBase world;
  parameter SI.Velocity vtTol = 0.01 "when to ignore aerodynamics";
  SI.Velocity vR_b[3] "relative air velocity resolved in body frame";
  SI.Acceleration aR_b[3] "relative air acceleration resolved in body frame";
  SI.Velocity vt "true air velocity";
  SI.Acceleration vtDot "true air velocity derivative";
  SI.Pressure qBar "dynamics pressure";
  SI.Angle alpha "angle of attack";
  SI.AngularVelocity alphaDot "derivative of angle of attack";
  SI.Angle beta "side slip angle";
  SI.AngularVelocity betaDot "derivative of side slip angle";

  parameter SI.Area s "reference area";
  parameter SI.Length cBar "average chord";
  parameter SI.Length b "span";

  constant Real epsilon = 1e-2;

  SI.AngularVelocity p "body roll rate";
  SI.AngularVelocity q "body pitch rate";
  SI.AngularVelocity r "body yaw rate";

  SI.Angle alpha_deg "angle of attack, deg";
  SI.Angle beta_deg "side slip, deg";

  Real C_bs[3,3] "stability to body frame";

  Real C_bw[3,3] "wind to body frame";

equation
  vR_b = fA.v_b - fA.C_br*world.wind_r(fA.r_r) + epsilon*ones(3);
  aR_b = der(vR_b);
  vt = sqrt(vR_b*vR_b);
  qBar = 0.5*world.rho(fA.r_r)*vt^2;
  alpha = atan2(vR_b[3],vR_b[1]);
  // omc doesn't like der(beta), setting manually
  alphaDot = (vR_b[1]*aR_b[3]-vR_b[3]*aR_b[1])/
    (vR_b[1]^2 + vR_b[3]^2); //stevens & lewis pg 78
  vtDot = (vR_b[1]*aR_b[1] + 
        vR_b[2]*aR_b[2] +
        vR_b[3]*aR_b[3])/vt;
  beta = asin(vR_b[2]/vt);
  // omc doesn't like der(beta), setting manually
  betaDot = (aR_b[2]*vt - vR_b[2]*vtDot) /
    vt*sqrt(vR_b[1]^2 + vR_b[3]^2);

  // alias's and conversions
  alpha_deg = SI.Conversions.to_deg(alpha);
  beta_deg = SI.Conversions.to_deg(alpha);
  {p,q,r} = fA.w_ib;
  // rotation matrices
  C_bs = Parts.T2(alpha);
  C_bw = Parts.T2(alpha)*Parts.T3(-beta);
end ForceMoment;

package StabilityFrame
  model ForceMoment
    extends Aerodynamics.ForceMoment;
    Real CL, CD, CY, Cl, Cm, Cn;
  equation
    F_b = C_bs*{-CD,-CY,-CL}*qBar*s;
    M_b = C_bs*{Cl*b,Cm*cBar,Cn*b}*qBar*s;
  end ForceMoment;
end StabilityFrame;

package WindFrame
  model ForceMoment
    extends Aerodynamics.ForceMoment;
    Real CL, CD, CC, Cl, Cm, Cn;
  equation
    F_b = C_bw*{-CD,-CC,-CL}*qBar*s;
    M_b = C_bw*{Cl*b,Cm*cBar,Cn*b}*qBar*s;
  end ForceMoment;
end WindFrame;

package BodyFrame
  model ForceMoment
    extends Aerodynamics.ForceMoment;
    Real CX, XY, CZ, Cl, Cm, Cn;
  equation
    F_b = {CX,CY,CZ}*qBar*s;
    M_b = {Cl*b,Cm*cBar,Cn*b}*qBar*s;
  end ForceMoment;
end BodyFrame;

// vim:ts=2:sw=2:expandtab:
