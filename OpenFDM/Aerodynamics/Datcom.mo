within OpenFDM.Aerodynamics;

package Datcom

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

  partial model ForceAndTorqueBase
    import Modelica.SIunits.Conversions.*;
    extends StabilityFrame.ForceAndTorque;
    extends CoefficientsAndDerivatives;

    Real alpha_deg;
    Real beta_deg;

    // controls
    Real dFlap, dElevator, dAileron;
      
  equation
    alpha_deg = to_deg(alpha);
    beta_deg = to_deg(beta);
    coefs.CL = CL_Basic +
         dCL_Flap * dFlap +
         dCL_Elevator * dElevator +
         dCL_PitchRate * q * coefs.cBar/(2*vt) +
         dCL_AlphaDot * alphaDot * coefs.cBar/(2*vt);
    coefs.CD = CD_Basic +
         dCD_Flap * dFlap +
         dCD_Elevator * dElevator;
    coefs.CY = dCY_Beta * beta +
         dCY_RollRate * p * coefs.b/(2*vt);
    coefs.Cl = dCl_Aileron * dAileron +
         dCl_Beta * beta +
         dCl_RollRate * p * coefs.b/(2*vt) +
         dCl_YawRate * r * coefs.b/(2*vt);   
    coefs.Cm = Cm_Basic +
         dCm_Flap * dFlap + 
         dCm_Elevator * dElevator +
         dCm_PitchRate * q * coefs.cBar/(2*vt) +
         dCm_AlphaDot * alphaDot * coefs.cBar/(2*vt);
    coefs.Cn = dCn_Aileron * dAileron +
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

end Datcom;

// vim:ts=2:sw=2:expandtab:
