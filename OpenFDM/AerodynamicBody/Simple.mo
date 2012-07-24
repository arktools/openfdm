within OpenFDM.AerodynamicBody;

model Simple

  extends CoefficientBased;

  // stall
  parameter Real alphaStall_deg = 20;

  // lift
  parameter Real cL0 = 0.1 "cl vs. aoa y-intercept";
  parameter Real cLa = 1.5/20.0 "cl vs. aoa slope";

  // drag
  parameter Real cD0 = 0.01 "drag polar y-intercept";
  parameter Real cDcL2 = 0.01 "cL^2 term in drag polar";

  // side force
  parameter Real cCb = 0.1/20.0 "side slip effect on side force";

  // roll moment
  parameter Real clp = 0.1 "roll damping";
  parameter Real cldA = 0.01/20.0 "aileron effect on roll";

  // pitch moment
  parameter Real cmq = 0.1 "pitch damping";
  parameter Real cma = 0.1 "aoa effect";
  parameter Real cmdE = 0.1/20.0 "elevator effect on pitch";

  // yaw moment
  parameter Real cnb = 1/20.0 "weather cocking stability";
  parameter Real cnr = 0.1 "yaw damping";
  parameter Real cndr = 0.1/20.0 "rudder effect on yaw";

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
