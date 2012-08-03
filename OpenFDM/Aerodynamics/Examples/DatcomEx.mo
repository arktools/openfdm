within OpenFDM.Aerodynamics.Examples;

model DatcomEx

  import MB=MultiBodyOmc;
  import OpenFDM.Aerodynamics.Datcom;
  import OpenFDM.Aerodynamics.Datcom.empty1D;
  import OpenFDM.Aerodynamics.Datcom.empty2D;

  constant Datcom.Tables datcomTables(    
      CL_Basic = empty1D,
      dCL_Flap  = empty1D,
      dCL_Elevator  = empty1D,
      dCL_PitchRate  = empty1D,
      dCL_AlphaDot  = empty1D,

      CD_Basic  = empty1D,
      dCD_Flap  = empty1D,
      dCD_Elevator  = empty1D,

      dCY_Beta  = empty1D,
      dCY_RollRate  = empty1D,

      dCl_Aileron  = empty1D,
      dCl_Beta  = empty1D,
      dCl_RollRate  = empty1D,
      dCl_YawRate  = empty1D,

      Cm_Basic = empty1D,
      dCm_Flap  = empty1D,
      dCm_Elevator  = empty1D,
      dCm_PitchRate  = empty1D,
      dCm_AlphaDot  = empty1D,

      dCn_Aileron  = empty1D,
      dCn_Beta  = empty1D,
      dCn_RollRate  = empty1D,
      dCn_YawRate  = empty1D);

  inner MB.World world(
    enableAnimation=false,
    n={0,0,1}
    );

  model Body 
    // find glide path of aircraft
    AirframeInitGlide airframe;
    Datcom.ForceAndTorque aerodynamics(
      animation=false,
      tables=datcomTables,
      rudder_deg = 0,
      flap_deg = 0,
      elevator_deg = 0,
      aileron_deg = 0,
      coefs(s=1, b=1, cBar=1));
  equation
    connect(airframe.frame_a,aerodynamics.frame_b);
  end Body;

  Body body;

end DatcomEx;

// vim:ts=2:sw=2:expandtab:
