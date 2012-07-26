package Datcom

model DatcomTable
    constant Real[:,:] CL_Basic;
end DatcomTable;

model DatcomCoefficients
    type PerDeg = Real(unit="1/deg");
    Real CL_Basic "basic lift coefficient";
    Real dCL_Flaps "change in lift coefficient due to flaps";
    Real dCL_Elevator "change in lift coefficient due to elevator";
    Real dCL_PitchRate "change in lift coefficient due to pitch rate";
    Real dCL_AlphaDot "change in lift coefficient due to aoa rate";
    Real CL "Total lift coefficient";
     
    Real CD_Basic "basic drag coefficient";
    Real dCD_Flaps "change in drag coefficient due to flaps";
    Real dCD_Elevator "change in drag coefficient due to elevator";
    Real CD "Total drag coefficient";

    Real dCY_Beta "change in side force coefficient due to side slip angle";
    Real dCY_RollRate "change in side force coefficient due to roll rate";
    Real CY_Total "Total side force coefficient";

    Real Cm_Basic;
    Real dCm_Flaps "change in pitch moment coefficient due to flaps";
    Real dCm_Elevator "change in pitch moment coefficient due to elevator";
    Real dCm_PitchRate "change in pitch moment coefficient due to pitch rate";
    Real dCm_AlphaDot "change in pitch moment coefficient due to aoa rate";
    Real Cm;

    Real dCl_Aileron "change in roll moment coefficient due to aileron";
    Real dCl_Beta "change in roll moment coefficient due to side slip angle";
    Real dCl_RollRate "change in roll moment coefficient ";
    Real dCl_YawRate;
end DatcomCoefficients;

model DatcomForceMoment
    DatcomTable table;
    extends DatcomCoefficients;
    CombiTable1Ds CL_Basic(table=.table.CL_Basic, u=alpha, y=CL_Basic);
end DatcomForceMoment;

end Datcom;
