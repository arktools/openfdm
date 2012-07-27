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

  // environment
  Environment env(alpha=alpha,beta=beta,p=p,q=q,r=r,qBar=qBar);

  Utilities.VariableRotation rotateAlpha(axis=2,angle=-env.alpha,angleDot=-env.alphaDot);
  Utilities.VariableRotation rotateBeta(axis=3,angle=env.beta,angleDot=env.betaDot);

  Real alpha;
  Real beta;
  Real alpha_deg;
  Real beta_deg;
  Real p;
  Real q;
  Real r;
  Real qBar;

equation

  // connect environment
  connect(env.frame,frame_b);

  // conversions
  alpha_deg = to_deg(alpha);
  beta_deg = to_deg(beta);

  // stability frame rotation
  connect(frame_b,rotateAlpha.frame_a);
  connect(rotateAlpha.frame_b,frame_stability);

  // wind frame rotation
  connect(frame_stability,rotateBeta.frame_a);
  connect(rotateBeta.frame_b,frame_wind);

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

  model SimpleForceAndTorque
    import Modelica.SIunits.Conversions.*;
    extends ForceAndTorque;

    // controls
    Real aileron_deg;
    Real elevator_deg;
    Real rudder_deg;

    // stall
    Real alphaStall_deg;

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

    alpha_deg_effective = stallModel(alpha_deg,alphaStall_deg);

    coefs.CL =
      CL0 +
      CLa*alpha_deg_effective +
      0;
    coefs.CD =
      CD0 +
      CDCL*coefs.CL^2 +
      0;
    coefs.CY =
      CYb*beta_deg +
      0;
    coefs.Cl =
      Clp*p +
      Clda*aileron_deg +
      0;
    coefs.Cm =
      Cma*alpha_deg_effective +
      Cmq*q +
      Cmde*elevator_deg +
      0;
    coefs.Cn = Cnb*beta_deg +
      Cnr*r +
      Cndr*rudder_deg +
      0;
  end SimpleForceAndTorque;


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
