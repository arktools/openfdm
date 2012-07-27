within OpenFDM.Aerodynamics.Examples;
model NullTable
  constant Real[2,2] empty1D = {{0,0},{0,0}}; 
  function ConstTable1D
    input Real c;
    output Real[2,2] table := {{0,c},{1,c}};
  end ConstTable1D;
  extends DatcomTables.DatcomCoefficientTableSet(
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
    dCn_YawRate  = empty1D
  );
end NullTable;


model DatcomTableStabilityFrameAeroObject
  import Aero=OpenFDM.Aerodynamics;
  constant NullTable tableData;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));

  Aero.StabilityFrame.DatcomTableForceAndTorque aerodynamics(
      dFlap = 0,
      dElevator = 0,
      dAileron = 0,
      tables=tableData,
      // controls
      coefs(s=1, b=1, cBar=1));
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
end DatcomTableStabilityFrameAeroObject;

model DatcomTableForceAndTorqueEx
  import MB=Modelica.Mechanics.MultiBody;
  inner MB.World world(n={0,0,1});
  DatcomTableStabilityFrameAeroObject body_s;
end DatcomTableForceAndTorqueEx;


// vim:ts=2:sw=2:expandtab:
