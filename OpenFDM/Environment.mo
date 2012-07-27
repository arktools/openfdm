within OpenFDM;

model Environment "environment for multibody frame"

  import SI = Modelica.SIunits;
  import Modelica.Math.Vectors;
  import Modelica.Mechanics.MultiBody;
  import Modelica.Mechanics.MultiBody.Frames.*;

  MultiBody.Interfaces.Frame frame;

  SI.Density rho "air density";  
  SI.Position asl "altitude above sea level";
  SI.Position agl "altitude above ground level";
  SI.Position groundAsl "altitude of ground above sea level";
  SI.Velocity wind_NED[3] "wind vector";

  // aerodynamic properties
  SI.Velocity vt "true airspeed";
  SI.Acceleration vtDot "Derivative of true airspeed";
  SI.Angle alpha "angle of attack";
  SI.AngularVelocity alphaDot "angle of attack derivative";
  SI.Angle beta "side slip angle";
  SI.Angle betaDot "side slip angle derivative";
  SI.Pressure qBar "average dynamics pressure";
  SI.AngularVelocity p "roll rate";
  SI.AngularVelocity q "pitch rate";
  SI.AngularVelocity r "yaw rate";

  SI.Velocity vRelative_NED[3];
  SI.Velocity vRelative_b[3];
  SI.Velocity aRelative_NED[3];
  SI.Velocity aRelative_b[3];

  SI.Velocity v_0[3];
  SI.Acceleration a_0[3];

  parameter Real vtTol  = 0.01 "when to set wind angles to zero to avoid singularity";

equation

  // TODO, adapt for lat, lon, alt
  v_0 = der(frame.r_0);
  a_0 = der(v_0);
  vRelative_NED = v_0 - wind_NED;
  aRelative_NED = a_0; // TODO: - der(wind_NED);
  vRelative_b = resolve2(frame.R,vRelative_NED);
  aRelative_b = resolve2(frame.R,aRelative_NED);
  vt = Vectors.norm(vRelative_b);
  {p,q,r} = angularVelocity2(frame.R);

  alpha = atan2(vRelative_b[3],vRelative_b[1]);
  qBar = 0.5*rho*vt^2;

  // avoid singularity in side slip angle calc
  if (vt > vtTol) then
    beta = asin(vRelative_b[2]/vt);
    betaDot = (aRelative_b[2]*vt - aRelative_b[2]*vtDot)/vt*sqrt(vRelative_b[1]^2 + vRelative_b[3]^2);
    vtDot = (vRelative_b[1]*aRelative_b[1] + 
      vRelative_b[2]*aRelative_b[2] +
      vRelative_b[3]*aRelative_b[3])/vt;
  else
    beta = 0;
    betaDot = 0;
    vtDot = 0;
  end if;

  // if negligible airspeed, set wind angles to zero
  // to avoid singularity
  if ( (vRelative_b[1]^2 + vRelative_b[3]^2) > vtTol) then
    alphaDot = (vRelative_b[1]*aRelative_b[3]-vRelative_b[3]*aRelative_b[1])/(vRelative_b[1]^2 + vRelative_b[3]^2); //stevens & lewis pg 78
  else
    alphaDot = 0;
  end if;

  assert(agl > 0, "altitude below ground level");
  asl = -frame.r_0[3]; // TODO: should subtract radius of earth
  agl = asl - groundAsl; 

  // TODO
  rho = 1.225;
  wind_NED = {0,0,0};
  groundAsl = 0;

  // envronment exerts no force torques directly
  // only provides state dependent info for frame
  frame.f = {0,0,0};
  frame.t = {0,0,0};

end Environment;

// vim:ts=2:sw=2:expandtab:
