within OpenFDM.Aerodynamics; 

record CoefficientsAndDerivativesSimple
  // stall
  Real alphaStall_deg "stall angle of attack";

  // lift
  Real CL0;
  Real CLa "CL alpha slope";

  // drag 
  Real CD0 "minimum drag";
  Real CDCL "CL^2 term for drag polar";

  // side force
  Real CYb "side slipe effect on side force";

  // roll moment
  Real Clp "roll damping, <0 for stability";
  Real Clda "aileron effect on roll";

  // pitch moment
  Real Cmq "pitch damping, <0 for stability";
  Real Cma "alpha effect on pitch, <0 for stability";
  Real Cmde "elevator effect on pitch";
  Real Cnb "weather cocking stability >0 for stability";
  Real Cnr "yaw damping, <0 for stability";
  Real Cndr "rudder effecto on yaw";
end CoefficientsAndDerivativesSimple;

record Controls
  Real aileron_deg;
  Real elevator_deg;
  Real rudder_deg;
  Real flap_deg;
end Controls;

function stallModel
  input Real angle;
  input Real stallAngle;
  output Real effective;
algorithm
  if (angle < stallAngle) then
    effective := angle;
  else // stall
    effective := 0;
  end if; 
end stallModel;

record MomentCoefficients
  Real Cl;
  Real Cm;
  Real Cn;
end MomentCoefficients;

record WingPlanform
  Real s;
  Real b;
  Real cBar;
end WingPlanform;

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

  constant Real epsilon = 1e-20;

  SI.AngularVelocity p "body roll rate";
  SI.AngularVelocity q "body pitch rate";
  SI.AngularVelocity r "body yaw rate";

  SI.Angle alpha_deg "angle of attack, deg";
  SI.Angle beta_deg "side slip, deg";

  Real C_bs[3,3] "stability to body frame";

  Real C_bw[3,3] "wind to body frame";

equation
  vR_b = fA.v_b - fA.C_br*world.wind_r(fA.r_r);
  vt = sqrt(vR_b*vR_b);
  qBar = 0.5*world.rho(fA.r_r)*vt^2;
  aR_b = der(vR_b);
  alpha = atan2(vR_b[3],(vR_b[1]+epsilon));
  alphaDot = der(alpha);
  vtDot = der(vt);
  beta = asin(vR_b[2]/vt);
  betaDot = der(beta);

  // alias's and conversions
  alpha_deg = SI.Conversions.to_deg(alpha);
  beta_deg = SI.Conversions.to_deg(beta);
  {p,q,r} = fA.w_ib;
  // rotation matrices
  C_bs = Parts.T2(alpha);
  C_bw = Parts.T2(alpha)*Parts.T3(beta);
end ForceMoment;

package StabilityFrame
  model ForceMoment
    extends Aerodynamics.ForceMoment;
    Real CL, CD, CY, Cl, Cm, Cn;
  equation
    F_b = C_bs*{-CD,-CY,-CL}*qBar*s;
    M_b = C_bs*{Cl*b,Cm*cBar,Cn*b}*qBar*s;
  end ForceMoment;


  model ForceMomentSimple
    extends ForceMoment;
    extends CoefficientsAndDerivativesSimple;
    extends Controls;
  protected
    Real alpha_deg_effective;
  equation
    alpha_deg_effective = stallModel(alpha_deg,alphaStall_deg);
    CL = CL0 + CLa*alpha_deg_effective;
    CD = CD0 + CDCL*coefs.CL^2;
    CY = CYb*beta_deg;
    Cl = Clp*p + Clda*aileron_deg;
    Cm = Cma*alpha_deg_effective +
      Cmq*q + Cmde*elevator_deg;
    Cn = Cnb*beta_deg +
      Cnr*r + Cndr*rudder_deg;
  end ForceMomentSimple;

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
