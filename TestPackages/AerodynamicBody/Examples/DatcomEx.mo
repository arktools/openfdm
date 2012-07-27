within OpenFDM.AerodynamicBody.Examples;

model DatcomEx
  import Modelica.SIunits.Conversions.from_deg;
  import Modelica.Blocks.Sources.Sine;
  import Modelica.Mechanics.MultiBody.World;

  inner World world(n={0,0,1});

  // sine generators for simulating pilot input
  Sine aileron(amplitude = 0.1, freqHz = 0.1);
  Sine elevator(amplitude = 0.1, freqHz = 0.1);
  Sine rudder(amplitude = 0.1, freqHz = 0.1);
  Sine flap(amplitude = 0.1, freqHz = 0.1);

  OpenFDM.AerodynamicBody.Datcom.AerodynamicBodyDatcom body(
    r={0,0,0}, // aerodynamic reference point
    r_CM={0,0,0}, // center of mass
    s=1, // wing area
    b=1, // span
    cBar=1, // avg. chord
    m=1, // mass
    I_11=1, I_22=1, I_33=1, // inertia
    r_0(start={0,0,-10000}, fixed=true), // position
    v_0(start={10,0,0}, fixed=true), // velocity
    angles_start=from_deg({0,0,0}));

equation

  connect(body.aileron, aileron.y);
  connect(body.elevator, elevator.y);
  connect(body.rudder, rudder.y);
  connect(body.flap, flap.y);

end DatcomEx;

// vim:ts=2:sw=2:expandtab:
