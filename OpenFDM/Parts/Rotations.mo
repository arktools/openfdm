within OpenFDM.Parts;

function T1
  import SI=Modelica.SIunits;
  input SI.Angle a;
  output Real T[3,3];
algorithm
  T := {{  1,      0,      0},
        {  0, cos(a), sin(a)},
        {  0,-sin(a), cos(a)}};
annotation(Inline=true);
end T1;

function T2
  import SI=Modelica.SIunits;
  input SI.Angle a;
  output Real T[3,3];
algorithm
  T := {{ cos(a),  0,-sin(a)},
        {  0,      1,      0},
        { sin(a),  0, cos(a)}};
annotation(Inline=true);
end T2;

function T3
  import SI=Modelica.SIunits;
  input SI.Angle a;
  output Real T[3,3];
algorithm
  T := {{ cos(a), sin(a), 0},
        {-sin(a), cos(a), 0},
        {      0,      0, 1}};
annotation(Inline=true);
end T3;

// vim:ts=2:sw=2:expandtab:
