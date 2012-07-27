within OpenFDM.AerodynamicBody;

partial model AerodynamicBody "a rigid body from the multibody library with aerodynamic force/torque to the frame_b connector"
  extends Modelica.Mechanics.MultiBody.Parts.BodyShape(
    r = aero_rp, // frame_b is attached to aero, this is aerodynamic reference point
    angles_fixed=true, // initial angles fixed
    w_0_fixed=true, // initial angular velocity fixed
    width=0.05, // visualization shape width
    // angle sequence typical for aero, body 3-2-1 
    sequence_angleStates = {3,2,1},
    sequence_start = sequence_angleStates,
    useQuaternions = false); // quaternions are causing simulation errors
  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  import Modelica.Mechanics.MultiBody.Frames.*;
  import Modelica.Mechanics.MultiBody.Interfaces;
  import Modelica.Blocks.Interfaces.RealInput;
  import Modelica.SIunits.Conversions.*;
  import OpenFDM.SIunits.Conversions.*;
  RealInput aileron;
  RealInput elevator;
  RealInput rudder;
  RealInput flap;
  parameter Real[3] aero_rp "aerodynamic reference point";
  parameter Real vtTol=0.001 "Velocity above which aerodynamics are enabled";
  parameter SI.AngularVelocity w_max = 2 "maximum rotational rate before structural failure";
  parameter Real g_max = 3 "maximum acceleration in g's before structural failure";
  
protected

  Interfaces.Frame stabilityFrame;
  Interfaces.Frame windFrame;
  Orientation R_sw "relative rotation object from stability into wind";

  Environment env;
  Modelica.Mechanics.MultiBody.Forces.WorldForceAndTorque forceTorque;

  SI.Velocity vt "true airspeed";
  SI.Acceleration vtDot "Derivative of true airspeed";
  SI.Angle alpha "angle of attack";
  SI.AngularVelocity alphaDot "angle of attack derivative";
  SI.Angle beta "side slip angle";
  SI.Angle betaDot "side slip angle derivative";
  SI.Pressure qBar "average dynamics pressure";
  SI.AngularVelocity aero_p "roll rate";
  SI.AngularVelocity aero_q "pitch rate";
  SI.AngularVelocity aero_r "yaw rate";

  SI.Velocity vRelative_ECEF[3];
  SI.Velocity vRelative_XYZ[3];
  SI.Velocity aRelative_ECEF[3];
  SI.Velocity aRelative_XYZ[3];

  Angle_deg roll_deg;
  Angle_deg pitch_deg;
  Angle_deg yaw_deg;

  Angle_deg alpha_deg;
  Angle_deg beta_deg;
  Angle_deg beta_deg_abs;
  Angle_deg aileron_deg;
  Angle_deg elevator_deg;
  Angle_deg flap_deg;
  Angle_deg rudder_deg;
  Length_ft agl_ft;

  Real accelNorm_g;
  Real wNorm;
  
equation

  // conversion
  yaw_deg = to_deg(body.phi[1]);
  pitch_deg = to_deg(body.phi[2]);
  roll_deg = to_deg(body.phi[3]);
  alpha_deg = to_deg(alpha); 
  beta_deg = to_deg(beta);
  beta_deg_abs = abs(beta_deg);
  flap_deg = to_deg(flap); 
  elevator_deg = to_deg(elevator); 
  aileron_deg = to_deg(aileron);
  rudder_deg = to_deg(rudder); 
  agl_ft = to_ft(env.agl);

  // structural failure check
  accelNorm_g =  Vectors.norm(a_0)/world.g;
  wNorm = Vectors.norm(angularVelocity2(frame_b.R));
  assert(accelNorm_g < g_max, "acceleration too high");
  assert(wNorm < w_max, "rotating too fast");

  // TODO
  vRelative_ECEF = v_0 - env.wind_ECEF;
  aRelative_ECEF = a_0; // TODO: - der(env.wind_ECEF);
  vRelative_XYZ = resolve2(frame_a.R,vRelative_ECEF);
  aRelative_XYZ = resolve2(frame_a.R,aRelative_ECEF);
  vt = Vectors.norm(vRelative_XYZ);
  {aero_p,aero_q,aero_r} = angularVelocity2(frame_b.R);

  alpha = atan2(vRelative_XYZ[3],vRelative_XYZ[1]);
  qBar = 0.5*env.rho*vt^2;

  // avoid singularity in side slip angle calc
  if (vt > vtTol) then
    beta = asin(vRelative_XYZ[2]/vt);
    betaDot = (aRelative_XYZ[2]*vt - aRelative_XYZ[2]*vtDot)/vt*sqrt(vRelative_XYZ[1]^2 + vRelative_XYZ[3]^2);
    vtDot = (vRelative_XYZ[1]*aRelative_XYZ[1] + 
      vRelative_XYZ[2]*aRelative_XYZ[2] +
      vRelative_XYZ[3]*aRelative_XYZ[3])/vt;
  else
    beta = 0;
    betaDot = 0;
    vtDot = 0;
  end if;

  // if negligible airspeed, set wind angles to zero
  // to avoid singularity
  if ( (vRelative_XYZ[1]^2 + vRelative_XYZ[3]^2) > vtTol) then
    alphaDot = (vRelative_XYZ[1]*aRelative_XYZ[3]-vRelative_XYZ[3]*aRelative_XYZ[1])/(vRelative_XYZ[1]^2 + vRelative_XYZ[3]^2); //stevens & lewis pg 78
  else
    alphaDot = 0;
  end if;

  connect(env.frame,frame_b);
  connect(frame_b,forceTorque.frame_b);
  // right hand set (forward, right, down)
  //TODO: change to equal datcom forcetorque

  stabilityFrame.R = axisRotation(2,-alpha, -alphaDot);
  stabilityFrame.r_0 = frame_b.r_0;
  R_sw = axisRotation(3,beta,betaDot);
  windFrame.R = absoluteRotation(stabilityFrame.R, R_sw);
  windFrame.r_0 = frame_b.r_0;

end AerodynamicBody;
// vim:ts=2:sw=2:expandtab:
