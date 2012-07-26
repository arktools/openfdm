package StabilityFrame

    model ForceMoment
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
    end ForceMoment;

end StabilityFrame;

package Datcom

model DatcomTable
    constant Real[:,:] CL_Basic;
end DatcomTable;

model DatcomCoefficients
    extends StabilityFrame.ForceMoment;
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
    Real alphaDot, beta;
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
end DatcomCoefficients;

model DatcomConstantEx
    extends DatcomCoefficients(
        p=1,
        q=1,
        r=1,
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

end Datcom;
