within OpenFDM.World;

model WorldBase

  import SI=Modelica.SIunits;

  function g_r
    input SI.Position r_r[3];
    output SI.Acceleration g_r[3];
  algorithm
    g_r := {0,0,9.8};
  annotation(Inline=true);
  end g_r;

  function agl
    input SI.Position r_r[3];
    output SI.Position agl;
  algorithm
    agl := -r_r[3];
  annotation(Inline=true);
  end agl;

  function rho
    input SI.Position r_r[3];
    output SI.Density rho;
  algorithm
    rho := 1.225;
  annotation(Inline=true);
  end rho;

  function wind_r
    input SI.Position r_r[3];
    output SI.Velocity wind_r[3];
  algorithm
    wind_r := {0,0,0};
  annotation(Inline=true);
  end wind_r;

end WorldBase;

// vim:ts=2:sw=2:expandtab
