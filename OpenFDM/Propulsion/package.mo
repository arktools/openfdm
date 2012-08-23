within OpenFDM;

package Propulsion

model Thruster
  extends Parts.ForceMoment;
  parameter Real maxThrust = 10;
  Modelica.Blocks.Nonlinear.Limiter sat(
    uMax=1,
    uMin=0);
  input Real throttle;
equation
  sat.u = throttle;
  F_b = sat.y*{maxThrust,0,0};
  M_b = {0,0,0};
end Thruster;


model SolidRocketMotor

  import SI=Modelica.SIunits;
  Interfaces.RigidConnector fA;

  parameter SI.Mass mInert = 0.1 "inert mass";
  parameter SI.Velocity Ve = 1000 "exit velocity";
  parameter SI.Mass mFuel = 0.1 "fuel mass";
  parameter SI.MassFlowRate mDot = 0.1 "mass flow rate";

  model Thrust
    extends Parts.ForceMoment;
    SI.MassFlowRate mDot;
    SI.Velocity Ve;
  equation
    F_b = {0,0,-mDot*Ve};
    M_b = {0,0,0}; 
  end Thrust;

  model Structure
    extends Parts.RigidBody;
    SI.Mass mInert;
    SI.Mass mFuel;
    SI.MassFlowRate mDot;
  equation
    der(mDot) = 0;
    der(mFuel) = -mDot;
    when (mFuel <= 0) then
      reinit(mFuel,0);
      reinit(mDot,0);
    end when;
    I_b = m*identity(3);
    m = mInert + mFuel;
  end Structure;

  Thrust thrust(Ve=Ve);
  Structure structure(mInert=mInert,
    mDot(start=mDot,fixed=true),
    mFuel(start=mFuel,fixed=true));

equation

  structure.mDot = thrust.mDot;
  connect(structure.fA,fA);
  connect(thrust.fA,structure.fA);

end SolidRocketMotor;


end Propulsion;

// vim:ts=2:sw=2:expandtab:
