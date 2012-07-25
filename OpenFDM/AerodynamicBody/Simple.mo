within OpenFDM.AerodynamicBody;

model Simple

  extends CoefficientBased;

  // stall
  Real alphaStall_deg;

  // lift
  Real cL0 "cl vs. aoa y-intercept";
  Real cLa "cl vs. aoa slope";

  // drag
  Real cD0 "drag polar y-intercept";
  Real cDcL2 "cL^2 term in drag polar";

  // side force
  Real cCb "side slip effect on side force";

  // roll moment
  Real clp "roll damping";
  Real cldA "aileron effect on roll";

  // pitch moment
  Real cmq "pitch damping";
  Real cma "aoa effect";
  Real cmdE "elevator effect on pitch";

  // yaw moment
  Real cnb "weather cocking stability";
  Real cnr "yaw damping";
  Real cndr "rudder effect on yaw";

protected

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

  Real alpha_deg_effective;

equation

  // modify effective alpha/ beta for stall model
  alpha_deg_effective = stallModel(
    alpha_deg,alphaStall_deg);

  cL = // lift  
    cLa*alpha_deg_effective + cL0 + // aoa effect
    0; // allows other lines to be commented out

  cD = // drag 
    cDcL2*cL^2 + cD0 + // drag polar
    0; // allows other lines to be commented out

  cC = // side force
    cCb*beta_deg+ // beta effect
    0; // allows other lines to be commented out

  cl = // roll moment 
    (-clp)*aero_p + // damping
    cldA*aileron_deg + // control input
    0; // allows other lines to be commented out

  cm = // pitch moment
    (-cma)*alpha_deg_effective + // aoa effect
    (-cmq)*aero_q + // damping
    cmdE*elevator_deg + // control input
    0; // allows other lines to be commented out

  cn = // yaw moment
    (cnb)*beta_deg + // weather cocking
    (-cnr)*aero_r + // damping 
    cndr*rudder_deg + // control input
    0; // allows other lines to be commented out

end Simple;

// vim:ts=2:sw=2:expandtab:
