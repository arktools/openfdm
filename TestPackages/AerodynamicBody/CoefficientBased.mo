within OpenFDM.AerodynamicBody;

model CoefficientBased "coefficient based aerodynamics"
  import SI = Modelica.SIunits;
  extends AerodynamicBody (forceTorque(frame_resolve=stabilityFrame));
  parameter SI.Area s "reference area";
  parameter SI.Length b "span";
  parameter SI.Length cBar "mean chord";
protected
  Real cL "lift coefficient";
  Real cD "drag coefficient";
  Real cC "cross-wind coefficient";
  Real cl "roll moment coefficient";
  Real cm "pitch moment coefficient";
  Real cn "yaw moment coefficient";
equation
  forceTorque.force = {-cD*qBar*s,-cC*qBar*s,-cL*qBar*s};
  forceTorque.torque = {cl*qBar*b*s,cm*qBar*cBar*s,cn*qBar*b*s};
end CoefficientBased;

// vim:ts=2:sw=2:expandtab:
