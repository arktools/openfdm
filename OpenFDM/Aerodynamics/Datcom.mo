within OpenFDM.Aerodynamics;

package Datcom

  record Controls
    Real flap_deg "flap";
    Real aileron_deg "aileron";
    Real elevator_deg "elevator";
    Real rudder_deg "rudder";
  end Controls;

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
    output Real y; 
    input Real u;
    constant Real table[:,:]; 
    Modelica.Blocks.Nonlinear.Limiter sat(
      uMax=table[size(table,1),1],
      uMin=table[1,1]);
    Modelica.Blocks.Tables.CombiTable1Ds combi(columns={2}, table=table);
  equation
    y = combi.y[1];
    sat.u = u;
    combi.u = sat.y;
  end CombiTable1DSISO;

  block CombiTable1DSIMO
    output Real y[:]; 
    input Real u;
    constant Real table[:,:]; 
    Modelica.Blocks.Nonlinear.Limiter sat(
      uMax=table[size(table,1),1],
      uMin=table[1,1]);
    Modelica.Blocks.Tables.CombiTable1Ds combi(table=table);
  equation
    y = combi.y;
    sat.u = u;
    combi.u = sat.y;
  end CombiTable1DSIMO;

  block CombiTable2DMISO
    output Real y;
    input Real u1, u2;
    constant Real table[:,:]; 
    Modelica.Blocks.Nonlinear.Limiter sat1(
      uMax=table[size(table,1),1],
      uMin=table[2,1]);
    Modelica.Blocks.Nonlinear.Limiter sat2(
      uMax=table[1,size(table,2)],
      uMin=table[1,2]);
    Modelica.Blocks.Tables.CombiTable2D combi(table=table);
  equation
    y = combi.y;
    sat1.u = u1;
    sat2.u = u2;
    combi.u1 = sat1.y;
    combi.u2 = sat2.y;
  end CombiTable2DMISO;

  partial model ForceMomentBase
    extends StabilityFrame.ForceMoment;
    extends CoefficientsAndDerivatives;
    extends Controls;
    constant Real rad2deg = 180.0/3.14159;

  equation
    CL = CL_Basic +
         dCL_Flap * flap_deg +
         dCL_Elevator * elevator_deg +
         dCL_PitchRate * rad2deg * q * cBar/(2*vt) +
         dCL_AlphaDot * rad2deg * alphaDot * cBar/(2*vt);
    CD = CD_Basic +
         dCD_Flap * flap_deg +
         dCD_Elevator * elevator_deg;
    CY = dCY_Beta * beta_deg +
         dCY_RollRate * rad2deg * p * b/(2*vt);
    Cl = dCl_Aileron * aileron_deg +
         dCl_Beta * beta_deg +
         dCl_RollRate * rad2deg * p * b/(2*vt) +
         dCl_YawRate * rad2deg * r * b/(2*vt);   
    Cm = Cm_Basic +
         dCm_Flap * flap_deg + 
         dCm_Elevator * elevator_deg +
         dCm_PitchRate * rad2deg * q * cBar/(2*vt) +
         dCm_AlphaDot * rad2deg * alphaDot * cBar/(2*vt);
    Cn = dCn_Aileron * aileron_deg +
        dCn_Beta * beta_deg +
         dCn_RollRate * rad2deg * p * b/(2*vt) +
         dCn_YawRate * rad2deg * r * b/(2*vt);  
  end ForceMomentBase;

  model ForceMoment
    import Modelica.Blocks.Tables.*;
    extends ForceMomentBase;
    constant Tables tables;
    CombiTable1DSISO CL_Basic_table(table=tables.CL_Basic, u=alpha_deg, y=CL_Basic);
    CombiTable1DSISO dCL_Flap_table(table=tables.dCL_Flap, u=flap_deg, y=dCL_Flap);
    CombiTable1DSISO dCL_Elevator_table(table=tables.dCL_Elevator, u=elevator_deg, y=dCL_Elevator);
    CombiTable1DSISO dCL_PitchRate_table(table=tables.dCL_PitchRate, u=alpha_deg, y=dCL_PitchRate);
    CombiTable1DSISO dCL_AlphaDot_table(table=tables.dCL_AlphaDot, u=alpha_deg, y=dCL_AlphaDot);
    CombiTable1DSISO CD_Basic_table(table=tables.CD_Basic, u=alpha_deg, y=CD_Basic);
    CombiTable2DMISO dCD_Flap_table(table=tables.dCD_Flap, u1=alpha_deg, u2=flap_deg, y=dCD_Flap);
    CombiTable2DMISO dCD_Elevator_table(table=tables.dCD_Elevator, u1=alpha_deg, u2=elevator_deg, y=dCD_Elevator);
    CombiTable1DSISO dCY_Beta_table(table=tables.dCY_Beta, u=alpha_deg, y=dCY_Beta);
    CombiTable1DSISO dCY_RollRate_table(table=tables.dCY_RollRate, u=alpha_deg, y=dCY_RollRate);
    CombiTable1DSISO dCl_Aileron_table(table=tables.dCl_Aileron, u=aileron_deg, y=dCl_Aileron);
    CombiTable1DSISO dCl_Beta_table(table=tables.dCl_Beta, u=alpha_deg, y=dCl_Beta);
    CombiTable1DSISO dCl_RollRate_table(table=tables.dCl_RollRate, u=alpha_deg, y=dCl_RollRate);
    CombiTable1DSISO dCl_YawRate_table(table=tables.dCl_YawRate, u=alpha_deg, y=dCl_YawRate);
    CombiTable1DSISO Cm_Basic_table(table=tables.Cm_Basic, u=alpha_deg, y=Cm_Basic);
    CombiTable1DSISO dCm_Flap_table(table=tables.dCm_Flap, u=alpha_deg, y=dCm_Flap);
    CombiTable1DSISO dCm_Elevator_table(table=tables.dCm_Elevator, u=alpha_deg, y=dCm_Elevator);
    CombiTable1DSISO dCm_PitchRate_table(table=tables.dCm_PitchRate, u=alpha_deg, y=dCm_PitchRate);
    CombiTable1DSISO dCm_AlphaDot_table(table=tables.dCm_AlphaDot, u=alpha_deg, y=dCm_AlphaDot);
    CombiTable2DMISO dCn_Aileron_table(table=tables.dCn_Aileron, u1=alpha_deg, u2=aileron_deg, y=dCn_Aileron);
    CombiTable1DSISO dCn_Beta_table(table=tables.dCn_Beta, u=alpha_deg, y=dCn_Beta);
    CombiTable1DSISO dCn_RollRate_table(table=tables.dCn_RollRate, u=alpha_deg, y=dCn_RollRate);
    CombiTable1DSISO dCn_YawRate_table(table=tables.dCn_YawRate, u=alpha_deg, y=dCn_YawRate);
  end ForceMoment;

  model ForceMomentCompact
    import Modelica.Blocks.Tables.*;
    extends ForceMomentBase;
    constant TablesCompact tables;
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

  end ForceMomentCompact;

  model ForceMomentSimple
    extends CoefficientsAndDerivativesSimple;
    extends ForceMomentBase(
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
      s=1, b=1, cBar=1);
    Real alpha_deg_effective;
  equation
    alpha_deg_effective = stallModel(alpha_deg,alphaStall_deg);
  end ForceMomentSimple;

end Datcom;

// vim:ts=2:sw=2:expandtab:
