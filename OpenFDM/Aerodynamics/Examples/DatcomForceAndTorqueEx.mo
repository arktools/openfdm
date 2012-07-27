within OpenFDM.Aerodynamics.Examples;

model DatcomStabilityFrameAeroObject
  import Aero=OpenFDM.Aerodynamics;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));
  Aero.StabilityFrame.DatcomForceAndTorque aerodynamics(
      // controls
        dFlap = 0,
        dElevator = 0,
        dAileron = 0,
        CL_Basic = 0,
        dCL_Flap = 0,
        dCL_Elevator = 0,
        dCL_PitchRate = 0,
        dCL_AlphaDot = 0,

        // drag force
        CD_Basic = 0,
        dCD_Flap = 0,
        dCD_Elevator = 0,

        // side force
        dCY_Beta = 0,
        dCY_RollRate = 0,

        // roll moment
        dCl_Aileron = 0,
        dCl_Beta = 0,
        dCl_RollRate = 0,
        dCl_YawRate = 0,

        // pitch moment
        Cm_Basic = 0,
        dCm_Flap = 0,
        dCm_Elevator = 0,
        dCm_PitchRate = 0,
        dCm_AlphaDot = 0,

        // yaw moment
        dCn_Aileron = 0,
        dCn_Beta = 0,
        dCn_RollRate = 0,
        dCn_YawRate = 0,
      coefs(s=1, b=1, cBar=1));
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
end DatcomStabilityFrameAeroObject;

model DatcomForceAndTorqueEx
  import MB=Modelica.Mechanics.MultiBody;
  inner MB.World world(n={0,0,1});
  DatcomStabilityFrameAeroObject body_s;
end DatcomForceAndTorqueEx;

// vim:ts=2:sw=2:expandtab:
