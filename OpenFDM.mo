package OpenFDM
  import Modelica.SIunits.*;
  record AircraftState
    Angle roll "roll";
    Angle pitch "pitch";
    Angle yaw "yaw";
    Angle lat "latitude";
    Angle lon "longitude";
    Distance alt "altitude above sea level";
    AngularVelocity p "roll rate";
    AngularVelocity q "pitch rate";
    AngularVelocity r "yaw rate";
    Angle alpha "angle of attack";
    AngularVelocity alphaDot "angle of attack rate";
    Angle beta "side slip angle";
  end AircraftState;
  record AerodynamicCoefficients
    Real cL;
    Real cB;
    Real cN;
  end AerodynamicCoefficients;
  model DatcomTable
    import Modelica.Blocks.Tables.*;
    AircraftState state;
    AerodynamicCoefficients coef;
    CombiTable1D cL(table = {{0,0},{1,1}});
    CombiTable1D cB(table = {{0,0},{1,1}});
    CombiTable2D cN(table = {{0,0,1},{0,0,1},{1,1,2}});
  equation
    connect(state.alpha,cL.u[1]);
    connect(coef.cL,cL.y[1]);
    connect(state.beta,cB.u[1]);
    connect(coef.cB,cB.y[1]);
    connect(state.alpha,cN.u1);
    connect(state.beta,cN.u2);
    connect(coef.cN,cN.y);
  end DatcomTable;
end OpenFDM;

