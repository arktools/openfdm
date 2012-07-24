within OpenFDM.AerodynamicBody.Examples;

model DatcomNull
  inner Modelica.Mechanics.MultiBody.World world(n={0,0,1});
  OpenFDM.AerodynamicBody.DatcomNull body(
    rudder = 0,
    aileron = 0,
    elevator = 0,
    flap = 0,
    aero_rp = {1,0,0},
    s = 1,
    b = 1,
    cBar = 1,
    m=1,
    I_11=1,
    I_22=1,
    I_33=1,
    r={0,0,0},
    r_CM={0,0,0},
    r_0(start={0,0,-10000}, fixed=true),
    v_0(start={10,0,0}, fixed=true),
    angles_start=from_deg({0,0,0}));
end DatcomNull;

// vim:ts=2:sw=2:expandtab:
