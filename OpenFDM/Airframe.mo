within OpenFDM;

model Airframe "this is a body shape with a 3-2-1 angle sequence common for aircraft, and with quaternions disabled by default due to numerical problems with the openmodelica solver and quaterions"
  import MB=Modelica.Mechanics.MultiBody;
  import Cv=Modelica.SIunits.Conversions;
  extends MB.Parts.Body(
    useQuaternions=false,
    sequence_angleStates = {3,2,1},
    sequence_start = sequence_angleStates);
  Real p "roll rate"; 
  Real pDot "roll rate derivative"; 
  Real q "pitch rate"; 
  Real qDot "pitch rate derivative"; 
  Real r "yaw rate"; 
  Real rDot "yaw rate derivative"; 
  Real roll_deg;
  Real pitch_deg;
  Real yaw_deg;

initial equation

  a_0 = {0,0,0};
  //a_0[3] = 0; // no accelerations in down frame, glide slope
  phi[1] = 0; // don't change yaw angle
  r_0 = {0,0,-10000}; // don't change position

equation
  yaw_deg = Cv.to_deg(phi[1]);
  pitch_deg = Cv.to_deg(phi[2]);
  roll_deg = Cv.to_deg(phi[3]);
  {p,q,r} = frame_a.R.w;
  {pDot,qDot,rDot} = der(frame_a.R.w);
end Airframe;

// vim:ts=2:sw=2:expandtab:
