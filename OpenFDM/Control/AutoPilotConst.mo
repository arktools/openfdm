within OpenFDM.Control;

model AutoPilotConst "a constant input autopilot"
  Real throttle(start=0.3, fixed=false);
  Real elevator_deg(start=0, fixed=false);
  Real rudder_deg(start=0, fixed=false);
  Real aileron_deg(start=0, fixed=false);
  Real flap_deg(start=0, fixed=true);
equation
  der(throttle) = 0;
  der(elevator_deg) = 0;
  der(rudder_deg) = 0;
  der(aileron_deg) = 0;
  der(flap_deg) = 0;
end AutoPilotConst;
// vim:ts=2:sw=2:expandtab
