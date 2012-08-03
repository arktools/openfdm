within OpenFDM.Aerodynamics;

package Datcom

  constant Real[2,2] empty1D = {{0,0},
                                {1,0}}; 
  constant Real[3,3] empty2D = {{0,0,1},
                                {0,0,0},
                                {1,0,0}};

  record Tables
    // lift
    Real[:,:] CL_Basic "basic lift coefficient";
    Real[:,:] dCL_Flap "change in lift coefficient due to flaps";
    Real[:,:] dCL_Elevator "change in lift coefficient due to elevator";
    Real[:,:] dCL_PitchRate "change in lift coefficient due to pitch rate";
    Real[:,:] dCL_AlphaDot "change in lift coefficient due to aoa rate";
    
    // drag 
    Real[:,:] CD_Basic "basic drag coefficient";
    Real[:,:] dCD_Flap "change in drag coefficient due to flaps";
    Real[:,:] dCD_Elevator "change in drag coefficient due to elevator";

    // sideforce
    Real[:,:] dCY_Beta "change in side force coefficient due to side slip angle";
    Real[:,:] dCY_RollRate "change in side force coefficient due to roll rate";

    // roll moment
    Real[:,:] dCl_Aileron "change in roll moment coefficient due to aileron";
    Real[:,:] dCl_Beta "change in roll moment coefficient due to side slip angle";
    Real[:,:] dCl_RollRate "change in roll moment coefficient due to roll rate";
    Real[:,:] dCl_YawRate "change in roll moment coefficient due to yaw rate";
    
    // pitch moment
    Real[:,:] Cm_Basic;
    Real[:,:] dCm_Flap "change in pitch moment coefficient due to flaps";
    Real[:,:] dCm_Elevator "change in pitch moment coefficient due to elevator";
    Real[:,:] dCm_PitchRate "change in pitch moment coefficient due to pitch rate";
    Real[:,:] dCm_AlphaDot "change in pitch moment coefficient due to aoa rate";
    
    // yaw moment
    Real[:,:] dCn_Aileron "change in yaw moment coefficient due to aileron";
    Real[:,:] dCn_Beta "change in yaw moment coefficient due to side slip angle";
    Real[:,:] dCn_RollRate "change in yaw moment coefficient due to roll rate";
    Real[:,:] dCn_YawRate "change in yaw moment coefficient due to yaw rate";
  end Tables;

  record TablesCompact
    type alphaColumns = enumeration(CL,CD); 
    Real[:,:] AlphaTable "coefficients as a function of alpha";
  end TablesCompact;

  record CoefficientsAndDerivatives
    // lift
    Real CL_Basic "basic lift coefficient";
    Real dCL_Flap "change in lift coefficient due to flaps";
    Real dCL_Elevator "change in lift coefficient due to elevator";
    Real dCL_PitchRate "change in lift coefficient due to pitch rate";
    Real dCL_AlphaDot "change in lift coefficient due to aoa rate";
     
    // drag
    Real CD_Basic "basic drag coefficient";
    Real dCD_Flap "change in drag coefficient due to flaps";
    Real dCD_Elevator "change in drag coefficient due to elevator";

    // sideforce
    Real dCY_Beta "change in side force coefficient due to side slip angle";
    Real dCY_RollRate "change in side force coefficient due to roll rate";

    // roll moment
    Real dCl_Aileron "change in roll moment coefficient due to aileron";
    Real dCl_Beta "change in roll moment coefficient due to side slip angle";
    Real dCl_RollRate "change in roll moment coefficient due to roll rate";
    Real dCl_YawRate "change in roll moment coefficient due to yaw rate";
    
    // pitch moment
    Real Cm_Basic "basic pitch moment coefficient";
    Real dCm_Flap "change in pitch moment coefficient due to flaps";
    Real dCm_Elevator "change in pitch moment coefficient due to elevator";
    Real dCm_PitchRate "change in pitch moment coefficient due to pitch rate";
    Real dCm_AlphaDot "change in pitch moment coefficient due to aoa rate";
    
    // yaw moment
    Real dCn_Aileron "change in yaw moment coefficient due to aileron";
    Real dCn_Beta "change in yaw moment coefficient due to side slip angle";
    Real dCn_RollRate "change in yaw moment coefficient due to roll rate";
    Real dCn_YawRate "change in yaw moment coefficient due to yaw rate";

  end CoefficientsAndDerivatives;

  block CombiTable1DSISO
    Real y1; 
    Real u1;
    extends Modelica.Blocks.Tables.CombiTable1Ds(columns={2});
  equation
    y[1] = y1; 
    u = u1;
  end CombiTable1DSISO;

  block CombiTable1DSIMO
    Real y1; 
    Real u1;
    extends Modelica.Blocks.Tables.CombiTable1Ds;
  equation
    y[1] = y1; 
    u = u1;
  end CombiTable1DSIMO;

  partial model ForceAndTorqueBase
    extends StabilityFrame.ForceAndTorque;
    extends CoefficientsAndDerivatives;
    extends Controls;

  equation
    coefs.CL = CL_Basic +
         dCL_Flap * flap_deg +
         dCL_Elevator * elevator_deg +
         dCL_PitchRate * q * coefs.cBar/(2*vt) +
         dCL_AlphaDot * alphaDot * coefs.cBar/(2*vt);
    coefs.CD = CD_Basic +
         dCD_Flap * flap_deg +
         dCD_Elevator * elevator_deg;
    coefs.CY = dCY_Beta * beta +
         dCY_RollRate * p * coefs.b/(2*vt);
    coefs.Cl = dCl_Aileron * aileron_deg +
         dCl_Beta * beta +
         dCl_RollRate * p * coefs.b/(2*vt) +
         dCl_YawRate * r * coefs.b/(2*vt);   
    coefs.Cm = Cm_Basic +
         dCm_Flap * flap_deg + 
         dCm_Elevator * elevator_deg +
         dCm_PitchRate * q * coefs.cBar/(2*vt) +
         dCm_AlphaDot * alphaDot * coefs.cBar/(2*vt);
    coefs.Cn = dCn_Aileron * aileron_deg +
        dCn_Beta * beta +
         dCn_RollRate * p * coefs.b/(2*vt) +
         dCn_YawRate * r * coefs.b/(2*vt);  
  end ForceAndTorqueBase;

  model ForceAndTorque
    import Modelica.Blocks.Tables.*;
    extends ForceAndTorqueBase;
    parameter Tables tables;
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
  end ForceAndTorque;

  model ForceAndTorqueCompact
    import Modelica.Blocks.Tables.*;
    extends ForceAndTorqueBase;
    parameter TablesCompact tables;
    CombiTable1DSIMO AlphaTable(table=tables.AlphaTable,y1=CL_Basic,u1=alpha);
  equation 
    // lift
    dCL_Flap = 0;
    dCL_Elevator = 0;
    dCL_PitchRate = 0;
    dCL_AlphaDot = 0;
     
    // drag
    CD_Basic = 0;
    dCD_Flap = 0;
    dCD_Elevator = 0;

    // sideforce
    dCY_Beta = 0;
    dCY_RollRate = 0;

    // roll moment
    dCl_Aileron = 0;
    dCl_Beta = 0;
    dCl_RollRate = 0;
    dCl_YawRate = 0;
    
    // pitch moment
    Cm_Basic = 0;
    dCm_Flap = 0;
    dCm_Elevator = 0;
    dCm_PitchRate = 0;
    dCm_AlphaDot = 0;
    
    // yaw moment
    dCn_Aileron = 0;
    dCn_Beta = 0;
    dCn_RollRate = 0;
    dCn_YawRate = 0;

  end ForceAndTorqueCompact;

  model ForceAndTorqueSimple
    extends CoefficientAndDerivativesSimple;
    // stall
    Real alpha_deg_effective;

    extends ForceAndTorqueBase(
      CL_Basic = CL0 + CLa*alpha_deg_effective,
      dCL_Flap = 0,
      dCL_Elevator = 0,
      dCL_PitchRate = 0,
      dCL_AlphaDot = 0,
      CD_Basic = CD0 + CDCL*CL_Basic^2,
      dCD_Flap = 0,
      dCD_Elevator = 0,
      dCY_Beta = CYb,
      dCY_RollRate = 0,
      dCl_Aileron = Clda,
      dCl_Beta = 0,
      dCl_RollRate = Clp,
      dCl_YawRate = 0,
      Cm_Basic = Cma*alpha_deg_effective,
      dCm_Flap = 0,
      dCm_Elevator = Cmde,
      dCm_PitchRate = Cmq,
      dCm_AlphaDot = 0,
      dCn_Aileron = 0,
      dCn_Beta = Cnb,
      dCn_RollRate = 0,
      dCn_YawRate = Cnr,
      coefs(s=1, b=1, cBar=1)
    );

  equation
    alpha_deg_effective = stallModel(alpha_deg,alphaStall_deg);
  end ForceAndTorqueSimple;

end Datcom;

// vim:ts=2:sw=2:expandtab:
