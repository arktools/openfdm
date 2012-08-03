within OpenFDM;

package Interfaces

  import SI=Modelica.SIunits;

  connector Frame
  " based on multibody"

    // translation
    flow SI.Force f_0[3];
    SI.Position r_0[3]; // w.r.t. cm

    // rotation
    flow SI.Torque t_0[3];
    Frames.Orientation R;
    //Real C_0f[3,3]; // rotation matrix from frame to body reference

  end Frame;

end Interfaces;

// vim:ts=2:sw=2:expandtab
