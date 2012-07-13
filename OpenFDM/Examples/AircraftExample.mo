within OpenFDM.Examples;
  
model AircraftExample
  AeroForceMoments aeroForceMoments;
  DatcomTable_F16 datcomTable;
  AircraftState state;
equation
  connect(aeroForceMoments.coeff,datcomTable.coeff);
  connect(aeroForceMoments.state,state);
  connect(datcomTable.state,state);
  state.alpha = 1;
end AircraftExample;

