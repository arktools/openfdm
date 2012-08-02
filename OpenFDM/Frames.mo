within OpenFDM;

package Frames

  import SI=Modelica.SIunits;

  function T1
    input Real a;
    output Real T[3,3];
  algorithm
    T := {{  1,      0,      0},
          {  0, cos(a), sin(a)},
          {  0,-sin(a), cos(a)}};
  annotation(Inline=true);
  end T1;

  function T2
    input Real a;
    output Real T[3,3];
  algorithm
    T := {{ cos(a),  0,-sin(a)},
          {  0,      1,      0},
          { sin(a),  0, cos(a)}};
  annotation(Inline=true);
  end T2;

  function T3
    input Real a;
    output Real T[3,3];
  algorithm
    T := {{ cos(a), sin(a), 0},
          {-sin(a), cos(a), 0},
          {      0,      0, 1}};
  annotation(Inline=true);
  end T3;

  /*model TranslationRotation*/
    /*Connectors.Frame frame_a;*/
    /*Connectors.Frame frame_b;*/
    /*Real r[3];*/
    /*Real C_ba[3,3];*/
  /*equation*/
    /*frame_b.C_0f = frame_a.C_0f*transpose(C_ba);*/
    /*frame_a.r_0 + frame_a.C_0f*r = frame_b.r_0;*/
    /*frame_a.f = -frame_b.C_0f*frame_b.f;*/
    /*frame_a.t = -frame_b.C_0f*frame_b.t;*/
  /*end TranslationRotation;*/

end Frames;

// vim:ts=2:sw=2:expandtab
