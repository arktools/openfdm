within OpenFDM.AerodynamicBody.Examples;

model CoefficientBasedEx
  import Modelica.Mechanics.MultiBody.World;
  import Modelica.SIunits.Conversion.*;
  inner World world(n={0,0,1});
  OpenFDM.AerodynamicBody.CoefficientBasedBlock body(
    rudder = 0,
    aileron = 0,
    elevator = 0,
    flap = 0,
    cL = 0,
    cD = 0,
    cC = 0,
    cl = 0,
    cm = 0,
    cn = 0,
    r = {0,0,0}, // aerodynamic reference point
    r_CM={0,0,0}, // center of mass
    s = 1,
    b = 1,
    cBar = 1,
    m=1,
    I_11=1, I_22=1, I_33=1,
    r_0(start={0,0,-10000}, fixed=true),
    v_0(start={10,0,0}, fixed=true),
    angles_start=from_deg({0,0,0}));
end CoefficientBasedEx;

// vim:ts=2:sw=2:expandtab:
