within OpenFDM.AerodynamicBody.Examples;

model SimpleEx
  import Modelica.SIunits.Conversions.from_deg;
  import Modelica.Blocks.Sources.Sine;
  import Modelica.Mechanics.MultiBody.World;

  inner World world(n={0,0,1});

  // sine generators for simulating pilot input
  Sine aileron(amplitude = 0.1, freqHz = 0.1);
  Sine elevator(amplitude = 0.1, freqHz = 0.1);
  Sine rudder(amplitude = 0.1, freqHz = 0.1);
  Sine flap(amplitude = 0.1, freqHz = 0.1);

  OpenFDM.AerodynamicBody.Simple body(
    alphaStall_deg=20,
    cL0=0.1,
    cLa=1.5/20.0,
    cD0=0.01,
    cDcL2=0.01,
    cCb=0.1/20.0,
    clp=0.1,
    cldA=0.01/20.0,
    cmq=0.1,
    cma=0.1,
    cmdE=0.1/20.0,
    cnb=1/20.0,
    cnr=0.1,
    cndr=0.1/20.0,
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

end SimpleEx;

// vim:ts=2:sw=2:expandtab:
