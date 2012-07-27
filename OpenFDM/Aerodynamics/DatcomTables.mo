within OpenFDM.Aerodynamics;

package DatcomTables

  record DatcomCoefficients
    Real CL_Basic "basic lift coefficient";
    Real dCL_Flap "change in lift coefficient due to flaps";
    Real dCL_Elevator "change in lift coefficient due to elevator";
    Real dCL_PitchRate "change in lift coefficient due to pitch rate";
    Real dCL_AlphaDot "change in lift coefficient due to aoa rate";
     
    Real CD_Basic "basic drag coefficient";
    Real dCD_Flap "change in drag coefficient due to flaps";
    Real dCD_Elevator "change in drag coefficient due to elevator";

    Real dCY_Beta "change in side force coefficient due to side slip angle";
    Real dCY_RollRate "change in side force coefficient due to roll rate";

    Real dCl_Aileron "change in roll moment coefficient due to aileron";
    Real dCl_Beta "change in roll moment coefficient due to side slip angle";
    Real dCl_RollRate "change in roll moment coefficient due to roll rate";
    Real dCl_YawRate "change in roll moment coefficient due to yaw rate";
    
    Real Cm_Basic;
    Real dCm_Flap "change in pitch moment coefficient due to flaps";
    Real dCm_Elevator "change in pitch moment coefficient due to elevator";
    Real dCm_PitchRate "change in pitch moment coefficient due to pitch rate";
    Real dCm_AlphaDot "change in pitch moment coefficient due to aoa rate";
    
    Real dCn_Aileron "change in yaw moment coefficient due to aileron";
    Real dCn_Beta "change in yaw moment coefficient due to side slip angle";
    Real dCn_RollRate "change in yaw moment coefficient due to roll rate";
    Real dCn_YawRate "change in yaw moment coefficient due to yaw rate";

  end DatcomCoefficients;

  model DatcomTables "Where values for datcom tables are set"
    constant Real[:,:] CL_Basic;
    constant Real[:,:] dCL_Flap;
    constant Real[:,:] dCL_Elevator;
    constant Real[:,:] dCL_PitchRate;
    constant Real[:,:] dCL_AlphaDot;

    constant Real[:,:] CD_Basic;
    constant Real[:,:] dCD_Flap;
    constant Real[:,:] dCD_Elevator;

    constant Real[:,:] dCY_Beta;
    constant Real[:,:] dCY_RollRate;

    constant Real[:,:] dCl_Aileron;
    constant Real[:,:] dCl_Beta;
    constant Real[:,:] dCl_RollRate;
    constant Real[:,:] dCl_YawRate;

    constant Real[:,:] Cm_Basic;
    constant Real[:,:] dCm_Flap;
    constant Real[:,:] dCm_Elevator;
    constant Real[:,:] dCm_PitchRate;
    constant Real[:,:] dCm_AlphaDot;

    constant Real[:,:] dCn_Aileron;
    constant Real[:,:] dCn_Beta;
    constant Real[:,:] dCn_RollRate;
    constant Real[:,:] dCn_YawRate;
  end DatcomTables;

  block CombiTable1DSISO
    Real y1; 
    Real u1;
    extends Modelica.Blocks.Tables.CombiTable1Ds(columns={2});
  equation
    y[1] = y1; 
    u = u1;
  end CombiTable1DSISO;

  model DatcomForceAndTorqueTable 
    import Modelica.Blocks.Tables.*;
    extends OpenFDM.Aerodynamics.StabilityFrame.DatcomForceAndTorque;
    parameter DatcomTables tables;

    CombiTable1DSISO CL_Basic_table(table=tables.CL_Basic, u1=alpha, y1=CL_Basic);
    CombiTable1DSISO dCL_Flap_table(table=tables.dCL_Flap, u1=alpha, y1=dCL_Flap);
    CombiTable1DSISO dCL_Elevator_table(table=tables.dCL_Elevator, u1=alpha, y1=dCL_Elevator);
    CombiTable1DSISO dCL_PitchRate_table(table=tables.dCL_PitchRate, u1=alpha, y1=dCL_PitchRate);
    CombiTable1DSISO dCL_AlphaDot_table(table=tables.dCL_AlphaDot, u1=alpha, y1=dCL_AlphaDot);
    CombiTable1DSISO CD_Basic_table(table=tables.CD_Basic, u1=alpha, y1=CD_Basic);
    CombiTable1DSISO dCD_Flap_table(table=tables.dCD_Flap, u1=alpha, y1=dCD_Flap);
    CombiTable1DSISO dCD_Elevator_table(table=tables.dCD_Elevator, u1=alpha, y1=dCD_Elevator);
    CombiTable1DSISO dCY_Beta_table(table=tables.dCY_Beta, u1=alpha, y1=dCY_Beta);
    CombiTable1DSISO dCY_RollRate_table(table=tables.dCY_RollRate, u1=alpha, y1=dCY_RollRate);
    CombiTable1DSISO dCl_Aileron_table(table=tables.dCl_Aileron, u1=alpha, y1=dCl_Aileron);
    CombiTable1DSISO dCl_Beta_table(table=tables.dCl_Beta, u1=alpha, y1=dCl_Beta);
    CombiTable1DSISO dCl_RollRate_table(table=tables.dCl_RollRate, u1=alpha, y1=dCl_RollRate);
    CombiTable1DSISO dCl_YawRate_table(table=tables.dCl_YawRate, u1=alpha, y1=dCl_YawRate);
    CombiTable1DSISO Cm_Basic_table(table=tables.Cm_Basic, u1=alpha, y1=Cm_Basic);
    CombiTable1DSISO dCm_Flap_table(table=tables.dCm_Flap, u1=alpha, y1=dCm_Flap);
    CombiTable1DSISO dCm_Elevator_table(table=tables.dCm_Elevator, u1=alpha, y1=dCm_Elevator);
    CombiTable1DSISO dCm_PitchRate_table(table=tables.dCm_PitchRate, u1=alpha, y1=dCm_PitchRate);
    CombiTable1DSISO dCm_AlphaDot_table(table=tables.dCm_AlphaDot, u1=alpha, y1=dCm_AlphaDot);
    CombiTable1DSISO dCn_Aileron_table(table=tables.dCn_Aileron, u1=alpha, y1=dCn_Aileron);
    CombiTable1DSISO dCn_Beta_table(table=tables.dCn_Beta, u1=alpha, y1=dCn_Beta);
    CombiTable1DSISO dCn_RollRate_table(table=tables.dCn_RollRate, u1=alpha, y1=dCn_RollRate);
    CombiTable1DSISO dCn_YawRate_table(table=tables.dCn_YawRate, u1=alpha, y1=dCn_YawRate);
  end DatcomForceAndTorqueTable;


  model ExampleDatcomTable
  function ConstTable1D
    input Real c;
    output Real[2,2] table := {{0,c},{1,c}};
  end ConstTable1D;
  extends DatcomTables.DatcomCoefficientTableSet(
    CL_Basic = ConstTable1D(1),
    dCL_Flap  = ConstTable1D(0),
    dCL_Elevator  = ConstTable1D(0),
    dCL_PitchRate  = ConstTable1D(0),
    dCL_AlphaDot  = ConstTable1D(0),

    CD_Basic  = ConstTable1D(0),
    dCD_Flap  = ConstTable1D(0),
    dCD_Elevator  = ConstTable1D(0),

    dCY_Beta  = ConstTable1D(0),
    dCY_RollRate  = ConstTable1D(0),

    dCl_Aileron  = ConstTable1D(0),
    dCl_Beta  = ConstTable1D(0),
    dCl_RollRate  = ConstTable1D(0),
    dCl_YawRate  = ConstTable1D(0),

    Cm_Basic = ConstTable1D(0),
    dCm_Flap  = ConstTable1D(0),
    dCm_Elevator  = ConstTable1D(0),
    dCm_PitchRate  = ConstTable1D(0),
    dCm_AlphaDot  = ConstTable1D(0),

    dCn_Aileron  = ConstTable1D(0),
    dCn_Beta  = ConstTable1D(0),
    dCn_RollRate  = ConstTable1D(0),
    dCn_YawRate  = ConstTable1D(0)
  );
end ExampleDatcomTable;
package NullAircraft
  
  constant Real[2,2] empty1D = {{0,0},{1,0}}; 
  constant DatcomTables datcomTables(    
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
end NullAircraft;



end DatcomTables;

// vim:ts=2:sw=2:expandtab:
