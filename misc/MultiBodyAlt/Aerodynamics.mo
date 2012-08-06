within OpenFDM;

package Aerodynamics

  import SI=Modelica.SIunits;
  import NonSI=Modelica.SIunits.Conversions.NonSIunits;

  record CoefficientAndDerivativesSimple
    // stall
    Real alphaStall_deg "stall angle of attack";

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
  end CoefficientAndDerivativesSimple;

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

  model ForceAndTorqueBase
    extends ForceAndTorque.Base;
    SI.Pressure qBar;
    SI.Angle alpha;
    SI.AngularVelocity alphaDot;
    SI.Angle beta;
    SI.AngularVelocity betaDot;
    SI.Velocity vt;
    SI.Acceleration vtDot;
    Real p, q, r;
    NonSI.Angle_deg alpha_deg;
    NonSI.Angle_deg beta_deg;
  equation
    alpha_deg = SI.Conversions.to_deg(alpha);  
    beta_deg = SI.Conversions.to_deg(beta);  
  end ForceAndTorqueBase;

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
      // TODO resolve in correct frame
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
      // TODO resolve in correct frame
      force = coefs.f;
      torque = coefs.t;
    end ForceAndTorque;

    model SimpleForceAndTorque

      import Modelica.SIunits.Conversions.*;
      extends ForceAndTorque;
      extends CoefficientAndDerivativesSimple;

      Real aileron_deg;
      Real elevator_deg;
      Real rudder_deg;
      Real flap_deg;

    protected

      Real alpha_deg_effective;

    equation

      alpha_deg_effective = stallModel(alpha_deg,alphaStall_deg);

      coefs.CL = CL0 + CLa*alpha_deg_effective;
      coefs.CD = CD0 + CDCL*coefs.CL^2;
      coefs.CY = CYb*beta_deg;
      coefs.Cl = Clp*p + Clda*aileron_deg;
      coefs.Cm = Cma*alpha_deg_effective +
        Cmq*q + Cmde*elevator_deg;
      coefs.Cn = Cnb*beta_deg +
        Cnr*r + Cndr*rudder_deg;

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
      Real f[3] = {-CD*qBar*s,-CC*qBar*s,-CL*qBar*s};
      Real t[3] = {Cl*qBar*s*b,Cm*qBar*s*cBar,Cn*qBar*s*b};
    end CoefficientEquations;

    model ForceAndTorque
      extends ForceAndTorqueBase;
      CoefficientEquations coefs(qBar=qBar);
    equation
      // TODO resolve in correct frame
      force = coefs.f;
      torque = coefs.t;
    end ForceAndTorque;
    
  end WindFrame;

end Aerodynamics;

// vim:ts=2:sw=2:expandtab:
