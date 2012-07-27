within OpenFDM.Aerodynamics;

partial model ForceAndTorqueBase
  "Aerodynamic force and torque acting at frame_b, this is a partial model
   that compiles useful data for aerodynamics, sets up frames"

  import SI = Modelica.SIunits;
  import MB = Modelica.Mechanics.MultiBody;
  import Modelica.Mechanics.MultiBody.Frames.*;
  import Modelica.Math.Vectors;
  import Modelica.SIunits.Conversions.*;
  import Modelica.Blocks.Interfaces.RealInput;

  extends MB.Forces.WorldForceAndTorque(resolveInFrame=MB.Types.ResolveInFrameB.frame_resolve);

  MB.Interfaces.Frame frame_wind;
  MB.Interfaces.Frame frame_stability;

  // parameters
  parameter Real vtTol = 0.001 "used to avoid singularities when approx. alphaDot etc.";

  // environment
  Environment env;

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

  Utilities.VariableRotation rotateAlpha(axis=2,angle=-alpha,angleDot=-alphaDot);
  Utilities.VariableRotation rotateBeta(axis=3,angle=beta,angleDot=betaDot);

equation

  // connect environment
  connect(env.frame,frame_b);

  connect(frame_b,rotateAlpha.frame_a);
  connect(rotateAlpha.frame_b,frame_stability);

  // wind frame rotation
  connect(frame_stability,rotateBeta.frame_a);
  connect(rotateBeta.frame_b,frame_wind);

  // TODO, adapt for lat, lon, alt
  v_0 = der(frame_b.r_0);
  a_0 = der(v_0);
  vRelative_NED = v_0 - env.wind_NED;
  aRelative_NED = a_0; // TODO: - der(env.wind_NED);
  vRelative_b = resolve2(frame_b.R,vRelative_NED);
  aRelative_b = resolve2(frame_b.R,aRelative_NED);
  vt = Vectors.norm(vRelative_b);
  {p,q,r} = angularVelocity2(frame_b.R);

  alpha = atan2(vRelative_b[3],vRelative_b[1]);
  qBar = 0.5*env.rho*vt^2;

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

end ForceAndTorqueBase;

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

record CoefficientEquationsBase
  extends WingPlanform;
  Real qBar;
end CoefficientEquationsBase;

package BodyFrame

  record Coefficients
    extends MomentCoefficients;
    Real CX;
    Real CY;
    Real CZ;
  end Coefficients;

  model CoefficientEquations
    extends Coefficients;
    extends CoefficientEquationsBase;
    Real f[3] = {CX*qBar*s,CY*qBar*s,CZ*qBar*s};
    Real t[3] = {Cl*qBar*s*b,Cm*qBar*s*cBar,Cn*qBar*s*b};
  end CoefficientEquations;

  model ForceAndTorque
    extends ForceAndTorqueBase;
    CoefficientEquations coefs(qBar=qBar);
  equation
    connect(frame_resolve,frame_b);
    force = coefs.f;
    torque = coefs.t;
  end ForceAndTorque;

end BodyFrame;

package StabilityFrame

  record Coefficients
    extends MomentCoefficients;
    Real CD;
    Real CY;
    Real CL;
  end Coefficients;

  model CoefficientEquations
    extends Coefficients;
    extends CoefficientEquationsBase;
    Real f[3] = {-CD*qBar*s,-CY*qBar*s,-CL*qBar*s};
    Real t[3] = {Cl*qBar*s*b,Cm*qBar*s*cBar,Cn*qBar*s*b};
  end CoefficientEquations;

  model ForceAndTorque
    extends ForceAndTorqueBase;
    CoefficientEquations coefs(qBar=qBar);
  equation
    connect(frame_resolve,frame_stability);
    force = coefs.f;
    torque = coefs.t;
  end ForceAndTorque;

end StabilityFrame;

package WindFrame

  record Coefficients
    extends MomentCoefficients;
    Real CD;
    Real CC;
    Real CL;
  end Coefficients;

  model CoefficientEquations
    extends Coefficients;
    extends CoefficientEquationsBase;
    //TODO fix these equations
    Real f[3] = {-CD*qBar*s,-CC*qBar*s,-CL*qBar*s};
    Real t[3] = {Cl*qBar*s*b,Cm*qBar*s*cBar,Cn*qBar*s*b};
  end CoefficientEquations;

  model ForceAndTorque
    extends ForceAndTorqueBase;
    CoefficientEquations coefs(qBar=qBar);
  equation
    connect(frame_resolve,frame_wind);
    force = coefs.f;
    torque = coefs.t;
  end ForceAndTorque;

end WindFrame;

// vim:ts=2:sw=2:expandtab:
