within OpenFDM.Aerodynamics.Examples;

model DatcomSimpleAeroObject
  import Aero=OpenFDM.Aerodynamics;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));

  // lift
  parameter Real CL0 = 0.1;
  parameter Real CLa = 0.1/20.0;
  // drag
  parameter Real CD0 = 0.01;
  parameter Real CDCL = 0.01;
  // side force
  parameter Real CYb = 0.01;
  // roll moment
  parameter Real Clp = -0.1;
  parameter Real Clda = 0.1;
  // pitch moment
  parameter Real Cmq = -0.1;
  parameter Real Cma = -0.1;
  parameter Real Cmde = 0.1;
  // yaw moment
  parameter Real Cnb = 0.1;
  parameter Real Cnr = -0.1;
  parameter Real Cndr = 0.1;

  Aero.StabilityFrame.DatcomForceAndTorque aerodynamics( 
    coefs(s=1, b=1, cBar=1)
  );
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
  aerodynamics.CL_Basic = CL0 + CLa*alpha_deg;
  aerodynamics.dCL_Flap = 0;
  aerodynamics.dCL_Elevator = 0;
  aerodynamics.dCL_PitchRate = 0;
  aerodynamics.dCL_AlphaDot = 0;
  aerodynamics.CD_Basic = CD0 + CDCL*aerodynamics.CL_Basic^2;
  aerodynamics.dCD_Flap = 0;
  aerodynamics.dCD_Elevator = 0;
  aerodynamics.dCY_Beta = CYb;
  aerodynamics.dCY_RollRate = 0;
  aerodynamics.dCl_Aileron = Clda;
  aerodynamics.dCL_Beta = 0;
  aerodynamics.dCL_RollRate = Clp;
  aerodynamics.dCL_YawRate = 0;
  aerodynamics.Cm_Basic = Cma*alpha_deg;
  aerodynamics.dCm_Flap = 0;
  aerodynamics.dCm_Elevator = Cmde;
  aerodynamics.dCm_PitchRate = Cmq;
  aerodynamics.dCm_AlphaDot = 0;
  aerodynamics.dCn_Aileron = 0;
  aerodynamics.dCn_Beta = Cnb;
  aerodynamics.dCn_RollRate = 0;
  aerodynamics.dCn_YawRate = Cnr;
end DatcomSimpleAeroObject;

model DatcomSimpleEx
  import MB=Modelica.Mechanics.MultiBody;
  inner MB.World world(n={0,0,1});
  SimpleStabilityFrameAeroObject body_s;
end DatcomSimpleEx;

// vim:ts=2:sw=2:expandtab:
