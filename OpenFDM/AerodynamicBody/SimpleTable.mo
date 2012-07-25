within OpenFDM.AerodynamicBody;

model SimpleTable

  parameter Real[:,:] table_cL0_alpha;

  extends Simple(
    alphaStall_deg=20,
    cLa=1.5/20.0,
    cD0=0.01,
    cDcL2=0.01,
    cCb=0.1/20.0,
    clp=0.1,
    cldA=0.01/20.0,
    cmq=0.1,
    cma=0.1,
    cmdE=0.1/20.0,
    cnb=1/20.0,
    cnr=0.1,
    cndr=0.1/20.0);

protected
  
  Modelica.Blocks.Tables.CombiTable1D cL0Table(
    u1=alpha_deg, y=cL0, table = table_cL0_alpha);

  Modelica.Blocks.Tables.CombiTable1D cLaTable(
    u1=alpha_deg, y=cLa, table = table_cLa_alpha);

  Modelica.Blocks.Tables.CombiTable1D cD0Table(
    u1=alpha_deg, y=cD0, table = table_cD0_alpha);



  Modelica.Blocks.Tables.CombiTable2D cL0Table(
    u1=alpha_deg, u2=beta_deg, y=cL0,
    table = table_cL0_alpha);


end SimpleTable;

// vim:ts=2:sw=2:expandtab:
