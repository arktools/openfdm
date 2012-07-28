within OpenFDM.AerodynamicBody.Examples;

model SimpleTableEx

  import Modelica.SIunits.Conversions.from_deg;
  import Modelica.Blocks.Sources.Sine;
  import Modelica.Mechanics.MultiBody.World;

  inner World world(n={0,0,1});

  // tables 
  Modelica.Blocks.Tables.CombiTable1D cL0Table(
    u1=alpha_deg, y=cL0, table = table_cL0_alpha);

  Modelica.Blocks.Tables.CombiTable1D cLaTable(
    u1=alpha_deg, y=cLa, table = table_cLa_alpha);

  Modelica.Blocks.Tables.CombiTable1D cD0Table(
    u1=alpha_deg, y=cD0, table = table_cD0_alpha);

  // sine generators for simulating pilot input
  Sine aileron(amplitude = 0.1, freqHz = 0.1);
  Sine elevator(amplitude = 0.1, freqHz = 0.1);
  Sine rudder(amplitude = 0.1, freqHz = 0.1);
  Sine flap(amplitude = 0.1, freqHz = 0.1);

  OpenFDM.AerodynamicBody.SimpleTable body(
    table_cL0_alpha=ConstTable1D(0.1),
    table_cLa_alpha=ConstTable1D(1.5/20.0),
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

protected

  function ConstTable1D
    input Real const;
    output Real[2,2] table = {{0,const},
                              {1,const}};
  end ConstTable1D;

  function ConstTable2D
    input Real const;
    output Real[2,2] table = {{0,    1},
                              {1,const}};
  end ConstTable2D;

equation

  connect(body.aileron, aileron.y);
  connect(body.elevator, elevator.y);
  connect(body.rudder, rudder.y);
  connect(body.flap, flap.y);

end SimpleTableEx;

// vim:ts=2:sw=2:expandtab:
