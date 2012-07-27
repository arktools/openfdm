within OpenFDM.Aerodynamics;

partial model ForceAndTorqueBase
  "Aerodynamic force and torque acting at frame_b, this is a partial model
   that compiles useful data for aerodynamics, sets up frames"

  import SI = Modelica.SIunits;
  import MB = Modelica.MultiBody;
  import Modelica.Math.Vectors;
  import Modelica.SIunits.Conversions.*;
  import Modelica.Blocks.Interfaces.RealInput;

  extends MB.Forces.WorldForceAndTorque(resolveInFrame=MB.Types.ResolveInB.frame_resolve);
  extends WingPlanform;

  MB.Interfaces.Frame frame_wind;
  MB.Interfaces.Frame frame_stability;

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

equation

  // connect environment to frame b
  connect(env.frame,frame_b);

  // stability frame
  frame_stability.R = absoluteRotation(frame_b.R,
    axisRotation(2,-alpha,-alpahDot));
  frame_stability.r_0 = frame_b.r_0;

  // wind frame
  frame_wind.R = absoluteRotation(frame_stability.R,
    axisRotation(3,beta,betaDot));
  frame_wind.r_0 = frame_b.r_0;

  // TODO, adapt for lat, lon, alt
  vRelative_NED = v_0 - env.wind_NED;
  aRelative_NED = a_0; // TODO: - der(env.wind_NED);
  vRelative_b = resolve2(frame_a.R,vRelative_NED);
  aRelative_b = resolve2(frame_a.R,aRelative_NED);
  vt = Vectors.norm(vRelative_b);
  {aero_p,aero_q,aero_r} = angularVelocity2(frame_b.R);

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
  extends MomentCoefficients;
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
    extends OpenFDM.Aerodynamics.Coefficients.BodyFrame;
    Real t[3] = {Cl*q*s,Cm*q*s,Cn*q*s};
    Real f[3] = {CX*q*s,CY*q*s,CZ*q*s};
  end CoefficientEquations;

  model ForceAndTorque
    extends ForceAndTorqueBase;
    extends WingPlanform;
    CoefficientEquations coefs(qBar=qBar,s=s,b=b,cBar=cBar);
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
    extends OpenFDM.Aerodynamics.Coefficients.Stabilityrame;
    Real t[3] = {Cl*q*s,Cm*q*s,Cn*q*s};
    Real f[3] = {-CD*q*s,-CC*q*s,-CL*q*s};
  end CoefficientEquations;

  model ForceAndTorque
    extends ForceAndTorqueBase;
    extends WingPlanform;
    CoefficientEquations coefs(qBar=qBar,s=s,b=b,cBar=cBar);
  equation
    connect(frame_resolve,stability_wind);
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
    extends OpenFDM.Aerodynamics.Coefficients.WindFrame;
    Real t[3] = {Cl*q*s,Cm*q*s,Cn*q*s};
    Real f[3] = {-CD*q*s,-CY*q*s,-CL*q*s};
  end CoefficientEquations;

  model ForceAndTorque
    extends ForceAndTorqueBase;
    extends WingPlanform;
    CoefficientEquations coefs(qBar=qBar,s=s,b=b,cBar=cBar);
  equation
    connect(frame_resolve,frame_wind);
    force = coefs.f;
    torque = coefs.t;
  end ForceAndTorque;

end WindFrame;

// vim:ts=2:sw=2:expandtab:
