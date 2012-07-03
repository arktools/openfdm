model OpenFDMTest
  import OpenFDM.*;
  AircraftState state(alpha = 1, beta = 1);
  AerodynamicCoefficients coef;
  DatcomTable table;
  constant Real q = 1;
  constant Real s = 1;
  Real L;
  Real cL = 1;
equation
  connect(table.state,state);
  connect(table.coef,coef);
  L = cL * q * s;
end OpenFDMTest;

