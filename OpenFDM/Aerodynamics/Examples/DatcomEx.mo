within OpenFDM.Aerodynamics.Examples;

package NullAircraft
  constant Real[2,2] empty1D = {{0,0},{1,0}}; 
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
end NullAircraft;


model DatcomAeroObject
  import Aero=OpenFDM.Aerodynamics;
  Airframe airframe(
    r_0(start={0,0,-10000}),
    v_0(start={10,0,0}));
  Aero.Datcom.ForceAndTorque aerodynamics(
    tables=NullAircraft.datcomTables,
    dFlap = 0,
    dElevator = 0,
    dAileron = 0,
    coefs(s=1, b=1, cBar=1));
equation
  connect(airframe.frame_a,aerodynamics.frame_b);
end DatcomAeroObject;

model DatcomEx
  import MB=Modelica.Mechanics.MultiBody;
  inner MB.World world(n={0,0,1});
  DatcomAeroObject body_s;
end DatcomEx;

// vim:ts=2:sw=2:expandtab:
