within OpenFDM.Aerodynamics;

record CoefficientAndDerivativesSimple
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
end CoefficientAndDerivativesSimple;

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

// vim:ts=2:sw=2:expandtab:
