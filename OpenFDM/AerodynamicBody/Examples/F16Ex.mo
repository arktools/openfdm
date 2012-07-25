within OpenFDM.AerodynamicBody.Examples;

model F16Ex
  import Modelica.Mechanics.MultiBody.World;
  import Modelica.SIunits.Conversion.from_deg;
  inner Modelica.Mechanics.MultiBody.World world(
    n={0,0,1});
  OpenFDM.AerodynamicBody.F16 body(
    rudder = 0,
    aileron = 0,
    elevator = 0,
    flap = 0,
    s = 27.87,
    b = 9.144,
    cBar = 3.45,
    m=9294.31,
    I_11=400.0,
    I_22=2352.0,
    I_33=2659,
    I_31=41.38,
    aero_rp={0,0,0},
    r_CM={0,0,0},
    r_0(start={0,0,-10000}, fixed=true),
    v_0(start={100,0,0}, fixed=true),
    angles_start=from_deg({0,0,0}),
    xcg = 0.35,
    xcgr = 0.4);
end F16Ex;

// vim:ts=2:sw=2:expandtab:
