within OpenFDM.Aerodynamics;
  
model DatcomTable
  import Modelica.Blocks.Tables.*;
  AeroCoefficients coeff;
  AircraftState state;
  parameter Boolean tableOnFile = false;
  CombiTable1D cLp;
equation
  connect(cLp.y[1],coeff.cLp);
  connect(cLp.u[1],state.alpha);
end DatcomTable;

