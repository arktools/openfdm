package OpenFDM
  model DatcomTable
    import Modelica.Blocks.Tables.*;
    AircraftState state;
    AerodynamicCoefficients coef;
    CombiTable1D cL(columns = {2}, table = {{0,0},{1,3}});
    CombiTable1D cB(columns = {2}, table = {{0,0},{1,3}});
  equation
    connect(state.alpha,cL.u[1]);
    connect(coef.cL,cL.y[1]);
    connect(state.beta,cB.u[1]);
    connect(coef.cB,cB.y[1]);
  end DatcomTable;
  record AircraftState
    //Real p "roll rate [deg/s]";
    //Real q "pitch0 rate [deg/s]";
    //Real r;
    Real alpha "angle of attach [deg]";
    //Real alphaDot;
    Real beta "side slip angle [deg]";
  end AircraftState;
  record AerodynamicCoefficients
    Real cL;
    Real cB;
  end AerodynamicCoefficients;
end OpenFDM;

