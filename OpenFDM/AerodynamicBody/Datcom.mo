within OpenFDM.AerodynamicBody;

package Datcom

package StabilityFrame

    model ForceAndTorque
       extends MultiBody.Forces.WorldForceAndTorque;
        Real CD;
        Real CL;
        Real CY;
        Real Cl;
        Real Cm;
        Real Cn;
        Real D;
        Real L;
        Real C;
        Real l;
        Real m;
        Real n;
        Real qBar, S, cBar, b;
    equation
        D = qBar * S * CD;
        L = qBar * S * CL;
        C = qBar * S * CY;
        l = qBar * S * b * Cl;
        m = qBar * S * cBar * Cm;
        n = qBar * S * b * Cn;
    end ForceAndTorque;

end StabilityFrame;



model DatcomCoefficientsForceAndTorque
    extends StabilityFrame.ForceAndTorque;
    import Modelica.Blocks.Interfaces.RealOutput;
    type PerDeg = Real(unit="1/deg");
    Real CL_Basic "basic lift coefficient";
    Real dCL_Flaps "change in lift coefficient due to flaps";
    Real dCL_Elevator "change in lift coefficient due to elevator";
    Real dCL_PitchRate "change in lift coefficient due to pitch rate";
    Real dCL_AlphaDot "change in lift coefficient due to aoa rate";
     
    Real CD_Basic "basic drag coefficient";
    Real dCD_Flaps "change in drag coefficient due to flaps";
    Real dCD_Elevator "change in drag coefficient due to elevator";

    Real dCY_Beta "change in side force coefficient due to side slip angle";
    Real dCY_RollRate "change in side force coefficient due to roll rate";

    Real dCl_Aileron "change in roll moment coefficient due to aileron";
    Real dCl_Beta "change in roll moment coefficient due to side slip angle";
    Real dCl_RollRate "change in roll moment coefficient due to roll rate";
    Real dCl_YawRate "change in roll moment coefficient due to yaw rate";
    
    Real Cm_Basic;
    Real dCm_Flaps "change in pitch moment coefficient due to flaps";
    Real dCm_Elevator "change in pitch moment coefficient due to elevator";
    Real dCm_PitchRate "change in pitch moment coefficient due to pitch rate";
    Real dCm_AlphaDot "change in pitch moment coefficient due to aoa rate";
    
    Real dCn_Aileron "change in yaw moment coefficient due to aileron";
    Real dCn_Beta "change in yaw moment coefficient due to side slip angle";
    Real dCn_RollRate "change in yaw moment coefficient due to roll rate";
    Real dCn_YawRate "change in yaw moment coefficient due to yaw rate";

    Real dFlaps, dElevator, dAileron;
    Real p, q, r;
    Real Vt;
    Real alphaDot, alpha, beta;
equation
    CL = CL_Basic +
         dCL_Flaps * dFlaps +
         dCL_Elevator * dElevator +
         dCL_PitchRate * q * cBar/(2*Vt) +
         dCL_AlphaDot * alphaDot * cBar/(2*Vt);
    CD = CD_Basic +
         dCD_Flaps * dFlaps +
         dCD_Elevator * dElevator;
    CY = dCY_Beta * beta +
         dCY_RollRate * p * b/(2*Vt);
    Cl = dCl_Aileron * dAileron +
         dCl_Beta * beta +
         dCl_RollRate * p * b/(2*Vt) +
         dCl_YawRate * r * b/(2*Vt);   
    Cm = Cm_Basic +
         dCm_Flaps * dFlaps + 
         dCm_Elevator * dElevator +
         dCm_PitchRate * q * cBar/(2*Vt) +
         dCm_AlphaDot * alphaDot * cBar/(2*Vt);
    Cn = dCn_Aileron * dAileron +
         dCn_Beta * beta +
         dCn_RollRate * p * b/(2*Vt) +
         dCn_YawRate * r * b/(2*Vt);   
end DatcomCoefficientsForceAndTorque;

model DatcomConstantEx
    extends DatcomCoefficientsForceAndTorque(
        p=1,
        q=1,
        r=1,
        alpha = 1,
        beta=1,
        alphaDot=1,
        Vt=1,
        cBar=1,
        b=1,
        qBar = 1,
        S = 1,
        dFlaps = 0,
        dAileron = 0,
        dElevator = 0,

        // lift force
        CL_Basic = 0,
        dCL_Flaps = 0,
        dCL_Elevator = 0,
        dCL_PitchRate = 0,
        dCL_AlphaDot = 0,

        // drag force
        CD_Basic = 0,
        dCD_Flaps = 0,
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
        dCm_Flaps = 0,
        dCm_Elevator = 0,
        dCm_PitchRate = 0,
        dCm_AlphaDot = 0,

        // yaw moment
        dCn_Aileron = 0,
        dCn_Beta = 0,
        dCn_RollRate = 0,
        dCn_YawRate = 0
    );
end DatcomConstantEx;

model DatcomCoefficientTableSet
    constant Real[:,:] CL_Basic;
    constant Real[:,:] dCL_Flaps;
    constant Real[:,:] dCL_Elevator;
    constant Real[:,:] dCL_PitchRate;
    constant Real[:,:] dCL_AlphaDot;

    constant Real[:,:] CD_Basic;
    constant Real[:,:] dCD_Flaps;
    constant Real[:,:] dCD_Elevator;

    constant Real[:,:] dCY_Beta;
    constant Real[:,:] dCY_RollRate;

    constant Real[:,:] dCl_Aileron;
    constant Real[:,:] dCl_Beta;
    constant Real[:,:] dCl_RollRate;
    constant Real[:,:] dCl_YawRate;

    constant Real[:,:] Cm_Basic;
    constant Real[:,:] dCm_Flaps;
    constant Real[:,:] dCm_Elevator;
    constant Real[:,:] dCm_PitchRate;
    constant Real[:,:] dCm_AlphaDot;

    constant Real[:,:] dCn_Aileron;
    constant Real[:,:] dCn_Beta;
    constant Real[:,:] dCn_RollRate;
    constant Real[:,:] dCn_YawRate;

end DatcomCoefficientTableSet;

block CombiTable1DSISO
  Real y1; 
  Real u1;
  extends Modelica.Blocks.Tables.CombiTable1Ds(columns={2});
equation
  y[1] = y1; 
  u = u1;
end CombiTable1DSISO;

model DatcomCoefficientTableSetForceAndTorque
  import Modelica.Blocks.Tables.*;
  extends DatcomCoefficientsForceAndTorque;

  CombiTable1DSISO CL_Basic_table(table=tables.CL_Basic, u1=alpha, y1=CL_Basic);
  CombiTable1DSISO dCL_Flaps_table(table=tables.dCL_Flaps, u1=alpha, y1=dCL_Flaps);
  CombiTable1DSISO dCL_Elevator_table(table=tables.dCL_Elevator, u1=alpha, y1=dCL_Elevator);
  CombiTable1DSISO dCL_PitchRate_table(table=tables.dCL_PitchRate, u1=alpha, y1=dCL_PitchRate);
  CombiTable1DSISO dCL_AlphaDot_table(table=tables.dCL_AlphaDot, u1=alpha, y1=dCL_AlphaDot);
  CombiTable1DSISO CD_Basic_table(table=tables.CD_Basic, u1=alpha, y1=CD_Basic);
  CombiTable1DSISO dCD_Flaps_table(table=tables.dCD_Flaps, u1=alpha, y1=dCD_Flaps);
  CombiTable1DSISO dCD_Elevator_table(table=tables.dCD_Elevator, u1=alpha, y1=dCD_Elevator);
  CombiTable1DSISO dCY_Beta_table(table=tables.dCY_Beta, u1=alpha, y1=dCY_Beta);
  CombiTable1DSISO dCY_RollRate_table(table=tables.dCY_RollRate, u1=alpha, y1=dCY_RollRate);
  CombiTable1DSISO dCl_Aileron_table(table=tables.dCl_Aileron, u1=alpha, y1=dCl_Aileron);
  CombiTable1DSISO dCl_Beta_table(table=tables.dCl_Beta, u1=alpha, y1=dCl_Beta);
  CombiTable1DSISO dCl_RollRate_table(table=tables.dCl_RollRate, u1=alpha, y1=dCl_RollRate);
  CombiTable1DSISO dCl_YawRate_table(table=tables.dCl_YawRate, u1=alpha, y1=dCl_YawRate);
  CombiTable1DSISO Cm_Basic_table(table=tables.Cm_Basic, u1=alpha, y1=Cm_Basic);
  CombiTable1DSISO dCm_Flaps_table(table=tables.dCm_Flaps, u1=alpha, y1=dCm_Flaps);
  CombiTable1DSISO dCm_Elevator_table(table=tables.dCm_Elevator, u1=alpha, y1=dCm_Elevator);
  CombiTable1DSISO dCm_PitchRate_table(table=tables.dCm_PitchRate, u1=alpha, y1=dCm_PitchRate);
  CombiTable1DSISO dCm_AlphaDot_table(table=tables.dCm_AlphaDot, u1=alpha, y1=dCm_AlphaDot);
  CombiTable1DSISO dCn_Aileron_table(table=tables.dCn_Aileron, u1=alpha, y1=dCn_Aileron);
  CombiTable1DSISO dCn_Beta_table(table=tables.dCn_Beta, u1=alpha, y1=dCn_Beta);
  CombiTable1DSISO dCn_RollRate_table(table=tables.dCn_RollRate, u1=alpha, y1=dCn_RollRate);
  CombiTable1DSISO dCn_YawRate_table(table=tables.dCn_YawRate, u1=alpha, y1=dCn_YawRate);
equation
  //connect(CL_Basic_table.y[1], CL_Basic);
end DatcomCoefficientTableSetForceAndTorque;

model DatcomTablesEx

    function ConstTable1D
      input Real c;
      output Real[2,2] table := {{0,c},{1,c}};
    end ConstTable1D;

    DatcomCoefficientTableSet tables(
      CL_Basic = ConstTable1D(0),
      dCL_Flaps  = ConstTable1D(0),
      dCL_Elevator  = ConstTable1D(0),
      dCL_PitchRate  = ConstTable1D(0),
      dCL_AlphaDot  = ConstTable1D(0),

      CD_Basic  = ConstTable1D(0),
      dCD_Flaps  = ConstTable1D(0),
      dCD_Elevator  = ConstTable1D(0),

      dCY_Beta  = ConstTable1D(0),
      dCY_RollRate  = ConstTable1D(0),

      dCl_Aileron  = ConstTable1D(0),
      dCl_Beta  = ConstTable1D(0),
      dCl_RollRate  = ConstTable1D(0),
      dCl_YawRate  = ConstTable1D(0),

      Cm_Basic = ConstTable1D(0),
      dCm_Flaps  = ConstTable1D(0),
      dCm_Elevator  = ConstTable1D(0),
      dCm_PitchRate  = ConstTable1D(0),
      dCm_AlphaDot  = ConstTable1D(0),

      dCn_Aileron  = ConstTable1D(0),
      dCn_Beta  = ConstTable1D(0),
      dCn_RollRate  = ConstTable1D(0),
      dCn_YawRate  = ConstTable1D(0)
    );

    extends DatcomCoefficientTableSetForceAndTorque(
        p=1,
        q=1,
        r=1,
        alpha = 1,
        beta=1,
        alphaDot=1,
        Vt=1,
        cBar=1,
        b=1,
        qBar = 1,
        S = 1,
        dFlaps = 0,
        dAileron = 0,
        dElevator = 0
    );
end DatcomTablesEx;

model DatcomAerodynamicBodyEx
end DatcomAerodynamicBodyEx;

model AerodynamicBodyDatcom
    extends AerodynamicBody(
        forceTorque=datcomForceTorque
    );
    DatcomCoefficientsForceAndTorque datcomForceTorque(
        resolveIn=stabilityFrame,
        p=p, // deg?
        q=q,
        r=r,
        alpha = alpha,
        beta=beta,
        alphaDot=alphaDot,
        Vt=vt,
        cBar=cBar,
        b=b,
        qBar = qBar,
        S = s,
        dFlaps = flaps,
        dAileron = aileron,
        dElevator = elevator
    );
end AerodynamicBodyDatcom;

end Datcom;

// vim:ts=2:sw=2:expandtab:
