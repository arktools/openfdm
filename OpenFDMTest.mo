model OpenFDMTest
  import OpenFDM.*;
  AircraftState state(lon = 0, alt = 0, roll = 0, pitch = 0, yaw = 0, p = 0, q = 0, r = 0, alpha = 1, alphaDot = 0, beta = 0);
  AerodynamicCoefficients coef;
  DatcomTable table;
  constant Real q = 1;
  constant Real s = 1;
  Real L;
  annotation(experiment(StartTime = 0.0, StopTime = 1.0, Tolerance = 0.000001));
equation
  connect(table.state,state);
  connect(table.coef,coef);
  L = coef.cL * q * s;
  der(state.lat) = L;
end OpenFDMTest;

